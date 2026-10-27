#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <config/config_loader.hpp>
#include <policy/observation/obs_collection.hpp>
#include <policy/runtime/policy_backend.hpp>
#include <utils/math.hpp>
#include <utils/utils.hpp>
#include <core/frog_simulation_core.hpp>

namespace
{
void require(bool ok, const char *message)
{
    if (!ok)
        throw std::runtime_error(message);
}

struct Result
{
    double time{0.0};
    double height{0.0};
    double distance{0.0};
    float max_action{0.0F};
};

// Use the real simulation sensor/PD bridge, deployment history collector and
// ONNX backend. No viewer, ROS transport or wall-clock timer is required.
Result rollout(const RobotConfig &robot, PolicyConfig policy,
               const std::filesystem::path &model_path,
               const std::filesystem::path &scene_path,
               const std::vector<float> &initial_joints,
               double duration = 10.0, float forward_command = 0.5F)
{
    char error[1024]{};
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
        mj_loadXML(scene_path.c_str(), nullptr, error, sizeof(error)), mj_deleteModel);
    if (!model)
        throw std::runtime_error(error);
    require(model->nu == policy.dof, "Scene joint count differs from policy");
    model->opt.timestep = 0.001;
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
    data->qpos[2] = 0.78;
    for (int i = 0; i < model->nu; ++i)
    {
        const int joint = model->actuator_trnid[2 * i];
        require(robot.joint_names[i] == mj_id2name(model.get(), mjOBJ_JOINT, joint),
                "Scene actuator order differs from hardware order");
        data->qpos[model->jnt_qposadr[joint]] = initial_joints[i];
    }
    mj_forward(model.get(), data.get());
    const auto sensors = frog::core::detect_sensor_layout(model.get());
    ObsCollection history;
    history.configure(policy);
    OnnxPolicyBackend backend;
    require(backend.load(policy, model_path), "Cannot load AMP ONNX model");
    std::vector<float> action(policy.dof, 0.0F);
    const double policy_dt = robot.control_dt * policy.decimation;
    const int substeps = static_cast<int>(std::lround(policy_dt / model->opt.timestep));
    Result result;
    for (int tick = 0; tick < static_cast<int>(duration / policy_dt); ++tick)
    {
        const auto state = frog::core::capture_state_frame(model.get(), data.get(), sensors, tick);
        std::vector<float> position, velocity;
        for (const auto &joint : state.joints)
        {
            position.push_back(static_cast<float>(joint.position));
            velocity.push_back(static_cast<float>(joint.velocity));
        }
        auto relative = utils::gather(position, policy.joint_mapping);
        for (std::size_t i = 0; i < relative.size(); ++i)
            relative[i] -= policy.default_dof_pos[i];
        history.set("ang_vel", std::vector<float>(state.imu.angular_velocity.begin(), state.imu.angular_velocity.end()));
        history.set("gravity_vec", math::gravity_from_quat(
            std::vector<float>(state.imu.orientation.begin(), state.imu.orientation.end())));
        history.set("commands", {tick * policy_dt < 3.0 ? 0.0F : forward_command, 0.0F, 0.0F});
        history.set("dof_pos", relative);
        history.set("dof_vel", utils::gather(velocity, policy.joint_mapping));
        history.set("actions", action);
        history.push();
        require(backend.infer(history.value(), action), "AMP inference failed during rollout");
        frog::bridge::CommandFrame command;
        command.joints.resize(policy.dof);
        for (std::size_t i = 0; i < action.size(); ++i)
        {
            require(std::isfinite(action[i]), "AMP action became non-finite");
            result.max_action = std::max(result.max_action, std::abs(action[i]));
            auto &joint = command.joints[policy.joint_mapping[i]];
            joint.position = policy.default_dof_pos[i] + action[i] * policy.action_scale[i];
            joint.kp = policy.kp[i];
            joint.kd = policy.kd[i];
        }
        for (int step = 0; step < substeps; ++step)
        {
            frog::core::apply_command_frame(model.get(), data.get(), command);
            mj_step(model.get(), data.get());
        }
        result.time = data->time;
        result.height = data->qpos[2];
        result.distance = std::hypot(data->qpos[0], data->qpos[1]);
        if (result.height < 0.4)
            break;
    }
    std::cout << "layout=" << policy.observation_history_priority
              << " time=" << result.time << " height=" << result.height
              << " distance=" << result.distance << " max_action=" << result.max_action << '\n';
    return result;
}
} // namespace

int main(int argc, char **argv)
{
    try
    {
        const std::filesystem::path root(Z_POLICY_ROOT);
        PolicyConfigLoader config;
        require(config.load(root, "g1"), "Cannot load G1 base config");
        const auto initial = config.robot().default_dof_pos;
        require(config.load(root, "g1", "amp"), "Cannot load AMP config");
        const auto path = config.policy_dir() / config.policy().model_name;
        const auto scene = root.parent_path() / "src/robots_model/g1/g1_mjcf/mjcf/scene_29dof.xml";
        if (argc == 2 && std::string(argv[1]) == "--zero-command")
        {
            bool holds_position = true;
            for (const auto &pose : {config.robot().default_dof_pos, initial})
            {
                const auto zero = rollout(config.robot(), config.policy(), path, scene, pose, 20.0, 0.0F);
                holds_position = holds_position && zero.time > 19.9 && zero.height > 0.6 && zero.distance < 0.1;
            }
            require(holds_position, "Zero command does not hold position within 0.1 m over 20 s; evaluate/retrain the checkpoint");
            return 0;
        }
        require(argc == 1, "Usage: test_amp_mujoco [--zero-command]");
        for (const auto &pose : {config.robot().default_dof_pos, initial})
        {
            const auto correct = rollout(config.robot(), config.policy(), path, scene, pose);
            require(correct.time > 9.9 && correct.height > 0.6, "Configured AMP policy falls in simulation");
            require(correct.distance > 1.0 && correct.max_action < 10.0F,
                    "Configured AMP policy does not respond to velocity command");
        }
        // Both layouts have 384 inputs; a shape-only smoke test cannot catch this.
        auto wrong = config.policy();
        wrong.observation_history_priority = "time";
        const auto broken = rollout(config.robot(), wrong, path, scene, config.robot().default_dof_pos);
        require(broken.time < 2.0, "Expected checkpoint layout regression was not reproduced");
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
