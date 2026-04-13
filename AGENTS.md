# Agent Notes - Loupe Flightdeck Plugin

## Critical Technical Lessons

### 1. X-Plane SDK Thread Safety
- **Rule:** NEVER call XPLM functions from background threads.
- **Reason:** The X-Plane SDK is strictly single-threaded. Calling functions like `XPLMDebugString` from a UDP listener thread causes memory corruption and crashes.
- **Solution:** Use `fprintf(stderr, ...)` for logging from background threads.

### 2. Thread Lifecycle & C++ Destruction
- **Rule:** Explicitly stop/join background threads in the **derived** class destructor.
- **Reason:** In C++, base class destructors run *after* derived class destructors. If the base class manages threads that call virtual functions (like `on_message`), those threads might try to access the destroyed derived class while the base class is still waiting to join them.
- **Solution:** Implement a `stop()` method and call it in the derived destructor.

### 3. Construction Safety
- **Rule:** Do not start background threads in a base class constructor if they call virtual methods.
- **Reason:** The virtual table is not fully set up for the derived class during the base class's construction phase.
- **Solution:** Use a separate `start()` method called after full construction (e.g., in `XPluginEnable`).

### 4. ImGui Lifecycle & Resource Cleanup
- **Rule:** Always explicitly call `ImGui_ImplOpenGL2_DestroyDeviceObjects()` (or equivalent for other backends) during shutdown, and ensure ImGui is destroyed **after** all X-Plane windows/loops are gone.
- **Reason:** The default ImGui backend implementation might leak textures (like fonts) if not explicitly cleaned up. Calling shutdown before destroying windows can lead to segmentation faults if X-Plane attempts to draw a frame while the context is being destroyed.
- **Solution:** Call `XPImGui::Shutdown()` at the very end of `XPluginDisable`.

### 5. Network Buffer Safety
- **Rule:** Never assume raw UDP buffers are null-terminated.
- **Reason:** Constructing a `std::string` or using `strcmp` on raw packet data can lead to buffer over-reads and crashes.
- **Solution:** Always use the packet length returned by `recvfrom` when creating strings or processing data (e.g., `std::string(buffer, len)`).

### 6. Simulator State Restoration
- **Rule:** Restore simulator state (weather, aircraft configuration) in `XPluginDisable`.
- **Reason:** Changes to global datarefs (like cloud bases) persist even after a plugin is disabled or reloaded. This creates "sticky" behavior that can confuse users or break other plugins.
- **Solution:** Store original values and apply a reset function during the disable/unload sequence.
