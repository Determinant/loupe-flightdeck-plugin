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
