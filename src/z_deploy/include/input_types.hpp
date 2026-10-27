#pragma once

#include <string>
#include <vector>


enum class joybutton
{
    None = 0,
    A,B,X,Y,
    LB,RB,LStick,RStick,
    DPadUp,DPadDown,DPadLeft,DPadRight,
    LB_A,LB_B,LB_X,LB_Y,
    LB_LStick,LB_RStick,
    LB_DPadUp,LB_DPadDown,LB_DPadLeft,LB_DPadRight,
    RB_A,RB_B,RB_X,RB_Y,
    RB_LStick,RB_RStick,
    RB_DPadUp,RB_DPadDown,RB_DPadLeft,RB_DPadRight,LB_RB
};

// Semantic stick state, already normalized to [-1, 1].
//   L_X -> axes[0], L_Y -> axes[1], R_X -> axes[3], R_Y -> axes[4]
struct joyaxis
{
    float L_X; // left stick horizontal: +1 right, -1 left
    float L_Y; // left stick vertical:   +1 down,  -1 up
    float R_X; // right stick horizontal: +1 right, -1 left
    float R_Y; // right stick vertical:   +1 down,  -1 up
};

struct JoySnapshot
{
    joybutton current{joybutton::None};
    joybutton previous{joybutton::None};
    joyaxis axis{};
};
