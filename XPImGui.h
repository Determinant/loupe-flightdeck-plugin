#pragma once

#include "XPLMDisplay.h"
#include <string>

namespace XPImGui {
    // Initialize the ImGui context and backend
    void Init();
    
    // Shutdown the ImGui context
    void Shutdown();

    // Reset internal state (call this during Init)
    void Reset();
    
    // Process X-Plane events and pass them to ImGui
    // Returns 1 if ImGui captured the event, 0 otherwise
    int HandleMouseClick(XPLMWindowID inWindowID, int x, int y, XPLMMouseStatus inMouseStatus, void *inRefcon);
    void HandleKey(XPLMWindowID inWindowID, char inKey, XPLMKeyFlags inFlags, char inVirtualKey, void *inRefcon, int losingFocus);
    
    // Begin a new frame (call this at the start of your draw callback)
    // Returns true if a new frame was started, false otherwise (e.g. if context is null)
    bool NewFrame(XPLMWindowID inWindowID);
    
    // Render the frame (call this at the end of your draw callback)
    void Render();
}
