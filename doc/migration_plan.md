# Migration Plan: Integrating `reflect_pytest` into `balancer_bot`

## 1. Executive Summary
The goal is to transition `balancer_bot` from a tightly coupled, callback-based architecture to a loosely coupled, message-driven architecture using the `MessageBus` IPC pattern from `reflect_pytest`. This enables Software-in-the-Loop (SIL) Python testing over UDP via C++26 static reflection, while ensuring the production Raspberry Pi build remains reliable.

To avoid breaking the existing project or fighting with experimental compiler setups globally, we will take an incremental, phased approach. We will keep the original `reflect_pytest` project intact as a reference and slowly port its ideas natively into `balancer_bot`.

## 2. Phase 1: Core IPC Infrastructure (The "Hub")
**Objective:** Integrate the core message bus and UDP bridge into the `balancer_bot` build system without migrating any active services.

- **Tasks:**
  - Create a new CMake library target (`ipc_hub`) in `balancer_bot/CMakeLists.txt` that compiles the source files purely from `reflect_pytest` (e.g., `src/message_bus.cpp`, `src/udp_bridge.cpp`, and `src/component_logger.cpp`).
  - Create a basic `messages/balancer_msgs.h` inside `balancer_bot`. This will define our `MsgId` enum and the core payload structures (e.g., `ImuData`, `MotorTargets`), but **without** any of the C++26 reflection `DOC_DESC` attributes or generator traits yet.
  - Verify that the new `ipc_hub` logic compiles perfectly under standard C++20 using the default system compiler.

## 3. Phase 2: Service Migration
**Objective:** Refactor existing `balancer_bot` services to use the `MessageBus` for data flow instead of `std::function` callbacks.

- **Tasks:**
  - **IMU Service:** Wrap `Ism330IioReader` in an `ImuService` class. Instead of triggering a callback, it will `publish<MsgId::ImuData>` to the bus.
  - **Control Service:** Wrap `RateControllerCore` in a `ControlService` class. It will subscribe to `MsgId::ImuData` and `MsgId::JoystickCommand`. On the control loop tick, it will publish `MsgId::MotorTargets` and `MsgId::SystemTelemetry`.
  - **Motor Service:** Wrap `MotorRunner` and the stepper logic in a `MotorService` class. It will subscribe to `MsgId::MotorTargets` and forward the commanded speeds to the hardware DMA logic.
  - Modify the old `main.cpp` logic to initialize the `MessageBus` and construct the `AppServices` tuple containing these wrapped services instead of manually wiring callbacks.

## 4. Phase 3: SIL Application and Testing
**Objective:** Enable isolated Software-in-the-Loop testing over UDP.

- **Tasks:**
  - Introduce `sil_app`, an executable that spins up the `MessageBus`, `UdpBridge`, and the new services (with mocked/SIL hardware interfaces where appropriate).
  - Copy over the Python `pytest` fixtures from `reflect_pytest`.
  - Ensure `sil_app` communicates correctly with the Python test harness over UDP (Port 9000).

## 5. Phase 4: C++26 Reflection and Code Generation
**Objective:** Re-introduce the automated Python bindings and Markdown documentation generation using the GCC trunk compiler.

- **Tasks:**
  - Annotate `messages/balancer_msgs.h` with the `DOC_DESC` macros required by the reflection system and add the missing message traits.
  - Add CMake targets for `generate_bindings` and `generate_docs`.
  - **Crucially**, isolate these generator targets so they *explicitly* compile with `/usr/local/gcc-trunk/bin/g++` and `-freflection`, ensuring the rest of `balancer_bot` is unaffected by the experimental toolchain.
  - Verify that `generated.py` is correctly built and usable by the `pytest` test suite.

## 6. Phase 5: Production Hardware Deployment
**Objective:** Finalize the Raspberry Pi build.

- **Tasks:**
  - Ensure `balancer_pi` uses the compiled `MessageBus` architecture on standard C++20.
  - Run the `balancer_pi` executable on real hardware.
  - Verify real-time performance on target hardware (latencies < 5ms) to ensure the MessageBus doesn't introduce unacceptable delay to the control loop.
