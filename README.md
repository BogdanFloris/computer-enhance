# Computer Enhance

Performance engineering exercises in C++.

## Build

```bash
cmake -B build -G Ninja
ninja -C build
```

## Run

```bash
./build/src/sim8086/sim8086
./build/src/haversine/haversine
```

## Test

After running `ning -C build`:

```bash
ctest --test-dir build --output-on-failure
```
