# biguit — minimal C++ project

Build and debug with VS Code (Linux):

Configure and build from the workspace root:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build --config Debug
```

Run locally:

```bash
./build/biguit
```

Notes:
- The project tries to link against SDL3 (pkg-config: `sdl3`) and Box2D (pkg-config: `box2d`).
- If those libraries are not installed, either install them or set `-DUSE_SDL3=OFF` or `-DUSE_BOX2D=OFF` when configuring.
