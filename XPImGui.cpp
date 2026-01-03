#include "XPImGui.h"
#include "imgui.h"
#include "imgui_impl_opengl2.h"
#include <XPLMUtilities.h>
#include <XPLMGraphics.h>
#include <XPLMProcessing.h>

// On Linux/Mac, X-Plane provides GL symbols.
#if LIN
    #include <GL/gl.h>
#elif __GNUC__
    #include <OpenGL/gl.h>
#else
    #include <gl/GL.h>
#endif

namespace XPImGui {

    void Init() {
        if (ImGui::GetCurrentContext()) return;
        Reset();
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGuiIO& io = ImGui::GetIO();
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
        io.IniFilename = nullptr;

        ImGui::StyleColorsDark();
        
        // Initialize OpenGL2 backend
        ImGui_ImplOpenGL2_Init(); 
    }

    void Shutdown() {
        if (!ImGui::GetCurrentContext()) return;
        ImGui_ImplOpenGL2_Shutdown();
        ImGui::DestroyContext();
    }
    
    // Store globals for mouse handling
    static float g_scale = 1.0f;
    static int g_screen_bounds_l = 0;
    static int g_screen_bounds_t = 0;
    static float g_last_scale = -1.0f;
    static float g_last_time = 0.0f;

    void Reset() {
        g_scale = 1.0f;
        g_screen_bounds_l = 0;
        g_screen_bounds_t = 0;
        g_last_scale = -1.0f;
        g_last_time = 0.0f;
    }

    bool NewFrame(XPLMWindowID inWindowID) {
        if (!ImGui::GetCurrentContext()) return false;

        // Get Logical Screen Bounds (Points)
        int sl, st, sr, sb;
        XPLMGetScreenBoundsGlobal(&sl, &st, &sr, &sb);
        g_screen_bounds_l = sl;
        g_screen_bounds_t = st;
        int screen_w_pts = sr - sl;
        
        // Get Physical Viewport (Pixels)
        GLint vp[4];
        glGetIntegerv(GL_VIEWPORT, vp);
        int screen_w_px = vp[2];
        int screen_h_px = vp[3];

        // Calculate HiDPI Scale
        float scale = (float)screen_w_px / (float)screen_w_pts;
        
        // Update ImGui Style Scaling if changed
        if (std::abs(scale - g_last_scale) > 0.001f) {
            ImGui::GetStyle().ScaleAllSizes(scale / (g_last_scale > 0 ? g_last_scale : 1.0f));
            ImGui::GetIO().FontGlobalScale = scale;
            g_last_scale = scale;
        }
        g_scale = scale;

        int l, t, r, b;
        XPLMGetWindowGeometry(inWindowID, &l, &t, &r, &b);
        int w = r - l;
        int h = t - b;
        
        ImGuiIO& io = ImGui::GetIO();
        io.DisplaySize = ImVec2((float)screen_w_px, (float)screen_h_px);
        
        float now = XPLMGetElapsedTime();
        io.DeltaTime = (g_last_time > 0) ? (now - g_last_time) : (1.0f / 60.0f);
        g_last_time = now;

        ImGui_ImplOpenGL2_NewFrame();
        ImGui::NewFrame();
        
        // Position window in Pixel Space
        // (Global_L - Screen_L) * Scale
        float imgui_x = (float)(l - sl) * scale;
        float imgui_y = (float)(st - t) * scale;
        
        ImGui::SetNextWindowPos(ImVec2(imgui_x, imgui_y));
        ImGui::SetNextWindowSize(ImVec2(w * scale, h * scale));
        ImGui::SetNextWindowBgAlpha(0.0f); 
        return true;
    }

    void Render() {
        if (!ImGui::GetCurrentContext()) return;

        ImGui::Render();
        
        // Save X-Plane's graphics state
        XPLMSetGraphicsState(0, 0, 0, 0, 1, 1, 0);
        
        ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
    }

    int HandleMouseClick(XPLMWindowID inWindowID, int x, int y, XPLMMouseStatus inMouseStatus, void *inRefcon) {
        if (!ImGui::GetCurrentContext()) return 0;

        ImGuiIO& io = ImGui::GetIO();
        
        // Mouse is in Global Points. Convert to Global Pixels (relative to viewport).
        // x_px = (x - sl) * scale
        // y_px = (st - y) * scale  [ImGui Y is down]
        
        float local_x = (float)(x - g_screen_bounds_l) * g_scale;
        float local_y = (float)(g_screen_bounds_t - y) * g_scale;
        io.AddMousePosEvent(local_x, local_y);

        if (inMouseStatus == xplm_MouseDown) {
            io.AddMouseButtonEvent(0, true);
        } else if (inMouseStatus == xplm_MouseUp) {
            io.AddMouseButtonEvent(0, false);
        }
        
        return io.WantCaptureMouse ? 1 : 0;
    }

    void HandleKey(XPLMWindowID inWindowID, char inKey, XPLMKeyFlags inFlags, char inVirtualKey, void *inRefcon, int losingFocus) {
        if (!ImGui::GetCurrentContext()) return;

        if (losingFocus) return;

        ImGuiIO& io = ImGui::GetIO();
        if (inFlags & xplm_DownFlag) {
            // Pass characters directly
            if (inKey != 0) {
                 io.AddInputCharacter(inKey);
            }
            
            // Map keys
            if (inVirtualKey == XPLM_VK_BACK) io.AddKeyEvent(ImGuiKey_Backspace, true);
            if (inVirtualKey == XPLM_VK_RETURN) io.AddKeyEvent(ImGuiKey_Enter, true);
            if (inVirtualKey == XPLM_VK_TAB) io.AddKeyEvent(ImGuiKey_Tab, true);
            if (inVirtualKey == XPLM_VK_LEFT) io.AddKeyEvent(ImGuiKey_LeftArrow, true);
            if (inVirtualKey == XPLM_VK_RIGHT) io.AddKeyEvent(ImGuiKey_RightArrow, true);
            if (inVirtualKey == XPLM_VK_UP) io.AddKeyEvent(ImGuiKey_UpArrow, true);
            if (inVirtualKey == XPLM_VK_DOWN) io.AddKeyEvent(ImGuiKey_DownArrow, true);
            if (inVirtualKey == XPLM_VK_HOME) io.AddKeyEvent(ImGuiKey_Home, true);
            if (inVirtualKey == XPLM_VK_END) io.AddKeyEvent(ImGuiKey_End, true);
            if (inVirtualKey == XPLM_VK_DELETE) io.AddKeyEvent(ImGuiKey_Delete, true);
            
        } else if (inFlags & xplm_UpFlag) {
             if (inVirtualKey == XPLM_VK_BACK) io.AddKeyEvent(ImGuiKey_Backspace, false);
            if (inVirtualKey == XPLM_VK_RETURN) io.AddKeyEvent(ImGuiKey_Enter, false);
            if (inVirtualKey == XPLM_VK_TAB) io.AddKeyEvent(ImGuiKey_Tab, false);
             if (inVirtualKey == XPLM_VK_LEFT) io.AddKeyEvent(ImGuiKey_LeftArrow, false);
            if (inVirtualKey == XPLM_VK_RIGHT) io.AddKeyEvent(ImGuiKey_RightArrow, false);
            if (inVirtualKey == XPLM_VK_UP) io.AddKeyEvent(ImGuiKey_UpArrow, false);
            if (inVirtualKey == XPLM_VK_DOWN) io.AddKeyEvent(ImGuiKey_DownArrow, false);
            if (inVirtualKey == XPLM_VK_HOME) io.AddKeyEvent(ImGuiKey_Home, false);
            if (inVirtualKey == XPLM_VK_END) io.AddKeyEvent(ImGuiKey_End, false);
            if (inVirtualKey == XPLM_VK_DELETE) io.AddKeyEvent(ImGuiKey_Delete, false);
        }
    }
}
