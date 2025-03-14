# Computer Enhance Codebase Guide

## Build Commands

- Build: `zig build`
- Run: `zig build run`
- Test all: `zig build test`
- Run specific test: `zig test src/decoder.zig -test-filter="MOV reg, reg/mem pattern"`
- Build and run assembly: `./asm/build_and_run.sh asm/file.asm`

## Code Style Guidelines

- Use snake_case for variable/function names
- Error handling with try/catch pattern
- 4-space indentation
- Explicit typing with var/const declarations
- Use structs with methods for logical grouping
- Prefer ArrayList for dynamic arrays
- Use descriptive enum names
- Consistent commenting with // style
- Format code with `zig fmt`
- Use switch statements with exhaustive patterns

## Architecture

- src/decoder.zig: 8086 instruction decoder
- src/main.zig: CLI interface for binary file processing
- asm/: Assembly code examples and testing utilities
