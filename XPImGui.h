#pragma once

#include "XPLMDisplay.h"

namespace XPImGui {
    // Initialize ImGui context. The OpenGL backend is initialized lazily in NewFrame().
    void Init();

    // Shutdown ImGui backend/context.
    void Shutdown();

    // Reset internal state used for input and frame timing.
    void Reset();

    // Forward X-Plane input events to ImGui.
    int HandleMouseClick(XPLMWindowID inWindowID, int x, int y, XPLMMouseStatus inMouseStatus, void *inRefcon);
    void HandleKey(XPLMWindowID inWindowID, char inKey, XPLMKeyFlags inFlags, char inVirtualKey, void *inRefcon, int losingFocus);

    // Begin a new ImGui frame from an X-Plane window draw callback.
    bool NewFrame(XPLMWindowID inWindowID);

    // Render current ImGui draw data.
    void Render();
}
