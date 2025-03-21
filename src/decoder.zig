// 8086 instruction decoder
const std = @import("std");
const testing = std.testing;
const mem = std.mem;
const assert = std.debug.assert;
const Allocator = mem.Allocator;
const ArrayList = std.ArrayList;

// Constants for instruction patterns
// MOV patterns
const MOV_REG_RM_PATTERN = 0x88;
const MOV_IMM_RM_PATTERN = 0xC6;
const MOV_IMM_REG_PATTERN = 0xB0;
const MOV_MEM_ACC_PATTERN = 0xA0;
const MOV_ACC_MEM_PATTERN = 0xA2;
const MOV_REG_RM_MASK = 0xFC;
const MOV_IMM_RM_MASK = 0xFE;
const MOV_IMM_REG_MASK = 0xF0;
const MOV_MEM_ACC_MASK = 0xFE;
const MOV_ACC_MEM_MASK = 0xFE;

// ADD patterns
const ADD_REG_RM_PATTERN = 0x00;
const ADD_IMM_RM_PATTERN = 0x80; // Note: shared with SUB and CMP (reg field distinguishes)
const ADD_IMM_ACC_PATTERN = 0x04;
const ADD_REG_RM_MASK = 0xFC;
const ADD_IMM_RM_MASK = 0xFC;
const ADD_IMM_ACC_MASK = 0xFE;

// SUB patterns
const SUB_REG_RM_PATTERN = 0x28;
const SUB_IMM_ACC_PATTERN = 0x2C;
const SUB_REG_RM_MASK = 0xFC;
const SUB_IMM_ACC_MASK = 0xFE;
// SUB uses same IMM_RM pattern (0x80) as ADD but with different reg field

// CMP patterns
const CMP_REG_RM_PATTERN = 0x38;
const CMP_IMM_ACC_PATTERN = 0x3C;
const CMP_REG_RM_MASK = 0xFC;
const CMP_IMM_ACC_MASK = 0xFE;
// CMP uses same IMM_RM pattern (0x80) as ADD and SUB but with different reg field

// Conditional jump patterns
const JE_PATTERN = 0x74;
const JL_PATTERN = 0x7C;
const JLE_PATTERN = 0x7E;
const JB_PATTERN = 0x72;
const JBE_PATTERN = 0x76;
const JP_PATTERN = 0x7A;
const JO_PATTERN = 0x70;
const JS_PATTERN = 0x78;
const JNE_PATTERN = 0x75;
const JNL_PATTERN = 0x7D;
const JG_PATTERN = 0x7F;
const JNB_PATTERN = 0x73;
const JA_PATTERN = 0x77;
const JNP_PATTERN = 0x7B;
const JNO_PATTERN = 0x71;
const JNS_PATTERN = 0x79;
const LOOP_PATTERN = 0xE2;
const LOOPZ_PATTERN = 0xE1;
const LOOPNZ_PATTERN = 0xE0;
const JCXZ_PATTERN = 0xE3;

// Reg field values for IMM_RM operations
const REG_FIELD_ADD = 0;
const REG_FIELD_SUB = 5;
const REG_FIELD_CMP = 7;

// Instruction set components
const Opcode = enum {
    mov,
    add,
    sub,
    cmp,
    je,
    jl,
    jle,
    jb,
    jbe,
    jp,
    jo,
    js,
    jne,
    jnl,
    jg,
    jnb,
    ja,
    jnp,
    jno,
    jns,
    loop,
    loopz,
    loopnz,
    jcxz,
    invalid,
};

const DecodeError = error{
    InsufficientBytes,
    InvalidOpcode,
};

const Instruction = struct {
    op: Opcode,
    d: u1, // direction  (0 - reg to rm, 1 - rm to reg)
    w: u1, // word/byte operation (0 - byte, 1 - word)
    mod: u2, // mode
    reg: u3, // register
    rm: u3, // register/memory
    s: u1 = 0, // sign extension flag (0 - full width, 1 - sign-extended byte)
    displacement: ?i16, // displacement can be signed
    immediate: ?i16, // immediate can be signed
    increment: ?i8, // loop increment
    consumedBytes: usize,

    // Register lookup tables
    fn registerName(reg: u3, w: u1) []const u8 {
        return switch (w) {
            0 => switch (reg) {
                0b000 => "al",
                0b001 => "cl",
                0b010 => "dl",
                0b011 => "bl",
                0b100 => "ah",
                0b101 => "ch",
                0b110 => "dh",
                0b111 => "bh",
            },
            1 => switch (reg) {
                0b000 => "ax",
                0b001 => "cx",
                0b010 => "dx",
                0b011 => "bx",
                0b100 => "sp",
                0b101 => "bp",
                0b110 => "si",
                0b111 => "di",
            },
        };
    }

    fn getRmName(rm: u3) []const u8 {
        return switch (rm) {
            0b000 => "[bx + si",
            0b001 => "[bx + di",
            0b010 => "[bp + si",
            0b011 => "[bp + di",
            0b100 => "[si",
            0b101 => "[di",
            0b110 => "[bp",
            0b111 => "[bx",
        };
    }

    // Helper function to sign extend 8-bit value to 16-bit
    fn signExtend8to16(value: u8) i16 {
        const signed: i8 = @bitCast(value);
        return signed;
    }

    // Helper function to read a signed 16-bit value
    fn readSigned16(bytes: []const u8, index: usize) i16 {
        const unsigned = @as(u16, bytes[index]) | (@as(u16, bytes[index + 1]) << 8);
        return @bitCast(unsigned);
    }

    pub fn decode(bytes: []const u8, startIndex: usize) !Instruction {
        var instruction = Instruction{
            .op = .invalid,
            .d = 0,
            .w = 0,
            .mod = 0b00,
            .reg = 0b000,
            .rm = 0b000,
            .displacement = null,
            .immediate = null,
            .increment = null,
            .consumedBytes = 0,
        };

        // MOV instructions
        if (try decodeMovRegRm(bytes, startIndex, &instruction)) {
            return instruction;
        }
        if (try decodeMovImmediateToReg(bytes, startIndex, &instruction)) {
            return instruction;
        }
        if (try decodeMovImmediateToRm(bytes, startIndex, &instruction)) {
            return instruction;
        }
        if (try decodeMovMemToAcc(bytes, startIndex, &instruction)) {
            return instruction;
        }
        if (try decodeMovAccToMem(bytes, startIndex, &instruction)) {
            return instruction;
        }

        // ADD instructions
        if (try decodeAddRegRm(bytes, startIndex, &instruction)) {
            return instruction;
        }
        if (try decodeAddImmToRm(bytes, startIndex, &instruction)) {
            return instruction;
        }
        if (try decodeAddImmToAcc(bytes, startIndex, &instruction)) {
            return instruction;
        }

        // SUB instructions
        if (try decodeSubRegRm(bytes, startIndex, &instruction)) {
            return instruction;
        }
        if (try decodeSubImmToRm(bytes, startIndex, &instruction)) {
            return instruction;
        }
        if (try decodeSubImmToAcc(bytes, startIndex, &instruction)) {
            return instruction;
        }

        // CMP instructions
        if (try decodeCmpRegRm(bytes, startIndex, &instruction)) {
            return instruction;
        }
        if (try decodeCmpImmToRm(bytes, startIndex, &instruction)) {
            return instruction;
        }
        if (try decodeCmpImmToAcc(bytes, startIndex, &instruction)) {
            return instruction;
        }

        // Conditional jump instructions
        if (try decodeConditionalJump(bytes, startIndex, &instruction)) {
            return instruction;
        }

        return instruction;
    }

    // Common functionality for decoding register/memory addressing
    fn decodeModRegRm(bytes: []const u8, startIndex: usize, offset: *usize, instruction: *Instruction) DecodeError!void {
        const modregrm = bytes[startIndex + 1];
        instruction.mod = @truncate(modregrm >> 6);
        instruction.reg = @truncate((modregrm & 0x38) >> 3);
        instruction.rm = @truncate(modregrm & 0x07);

        offset.* = 2;
        switch (instruction.mod) {
            0b00 => {
                if (instruction.rm == 0b110) {
                    if (bytes.len < startIndex + offset.* + 2) {
                        return error.InsufficientBytes;
                    }
                    // direct address 16-bit displacement
                    instruction.displacement = readSigned16(
                        bytes,
                        startIndex + offset.*,
                    );
                    offset.* += 2;
                }
            },
            0b01 => {
                if (bytes.len < startIndex + offset.* + 1) {
                    return error.InsufficientBytes;
                }
                // 8-bit displacement with sign extension
                instruction.displacement = signExtend8to16(
                    bytes[startIndex + offset.*],
                );
                offset.* += 1;
            },
            0b10 => {
                if (bytes.len < startIndex + offset.* + 2) {
                    return error.InsufficientBytes;
                }
                // 16-bit displacement
                instruction.displacement = readSigned16(
                    bytes,
                    startIndex + offset.*,
                );
                offset.* += 2;
            },
            0b11 => {},
        }
    }

    // Common functionality for decoding immediate values
    fn decodeImmediate(bytes: []const u8, startIndex: usize, offset: *usize, instruction: *Instruction) DecodeError!void {
        // For sign-extended instructions, decode based on s and w flags
        if (instruction.s == 1 and instruction.w == 1) {
            // Sign-extended 8-bit immediate to 16-bit
            if (bytes.len < startIndex + offset.* + 1) {
                return error.InsufficientBytes;
            }
            instruction.immediate = signExtend8to16(bytes[startIndex + offset.*]);
            offset.* += 1;
        } else if (instruction.w == 1) {
            // Full 16-bit immediate
            if (bytes.len < startIndex + offset.* + 2) {
                return error.InsufficientBytes;
            }
            instruction.immediate = readSigned16(bytes, startIndex + offset.*);
            offset.* += 2;
        } else {
            // 8-bit immediate
            if (bytes.len < startIndex + offset.* + 1) {
                return error.InsufficientBytes;
            }
            instruction.immediate = signExtend8to16(bytes[startIndex + offset.*]);
            offset.* += 1;
        }
    }

    fn decodeMovRegRm(bytes: []const u8, startIndex: usize, instruction: *Instruction) DecodeError!bool {
        if (bytes[startIndex] & MOV_REG_RM_MASK != MOV_REG_RM_PATTERN) {
            return false;
        }

        const opcodedw = bytes[startIndex];
        instruction.op = .mov;
        instruction.d = if (opcodedw & 0x02 == 0) 0 else 1;
        instruction.w = if (opcodedw & 0x01 == 0) 0 else 1;

        var offset: usize = 0;
        try decodeModRegRm(bytes, startIndex, &offset, instruction);
        instruction.consumedBytes = offset;
        return true;
    }

    fn decodeMovImmediateToRm(
        bytes: []const u8,
        startIndex: usize,
        instruction: *Instruction,
    ) DecodeError!bool {
        if (bytes[startIndex] & MOV_IMM_RM_MASK != MOV_IMM_RM_PATTERN) {
            return false;
        }
        instruction.op = .mov;
        instruction.w = if (bytes[startIndex] & 0x01 == 0) 0 else 1;

        var offset: usize = 0;
        try decodeModRegRm(bytes, startIndex, &offset, instruction);
        instruction.reg = 0; // For MOV immediate, reg field should be 0

        try decodeImmediate(bytes, startIndex, &offset, instruction);
        instruction.consumedBytes = offset;
        return true;
    }

    fn decodeMovImmediateToReg(
        bytes: []const u8,
        startIndex: usize,
        instruction: *Instruction,
    ) DecodeError!bool {
        if (bytes[startIndex] & MOV_IMM_REG_MASK != MOV_IMM_REG_PATTERN) {
            return false;
        }

        instruction.op = .mov;
        instruction.d = 1;
        const opcodewreg = bytes[startIndex];
        instruction.w = if (opcodewreg & 0x08 == 0) 0 else 1;
        instruction.reg = @truncate(opcodewreg & 0x07);
        instruction.mod = 0b11;

        var offset: usize = 1; // Skip the first byte
        try decodeImmediate(bytes, startIndex, &offset, instruction);
        instruction.consumedBytes = offset;
        return true;
    }

    fn decodeMovMemToAcc(
        bytes: []const u8,
        startIndex: usize,
        instruction: *Instruction,
    ) DecodeError!bool {
        if (bytes[startIndex] & MOV_MEM_ACC_MASK != MOV_MEM_ACC_PATTERN) {
            return false;
        }
        instruction.op = .mov;
        instruction.w = if (bytes[startIndex] & 0x01 == 0) 0 else 1;
        instruction.d = 1; // Always accumulator as destination
        instruction.mod = 0;
        instruction.reg = 0; // Accumulator
        instruction.rm = 0b110; // Direct address mode

        if (bytes.len < startIndex + 3) {
            return error.InsufficientBytes;
        }
        instruction.displacement = readSigned16(bytes, startIndex + 1);
        instruction.consumedBytes = 3;
        return true;
    }

    fn decodeMovAccToMem(
        bytes: []const u8,
        startIndex: usize,
        instruction: *Instruction,
    ) DecodeError!bool {
        if (bytes[startIndex] & MOV_ACC_MEM_MASK != MOV_ACC_MEM_PATTERN) {
            return false;
        }
        instruction.op = .mov;
        instruction.w = if (bytes[startIndex] & 0x01 == 0) 0 else 1;
        instruction.d = 0; // Always memory as destination
        instruction.mod = 0;
        instruction.reg = 0; // Accumulator
        instruction.rm = 0b110; // Direct address mode

        if (bytes.len < startIndex + 3) {
            return error.InsufficientBytes;
        }
        instruction.displacement = readSigned16(bytes, startIndex + 1);
        instruction.consumedBytes = 3;
        return true;
    }

    // ADD instruction decoders
    fn decodeAddRegRm(bytes: []const u8, startIndex: usize, instruction: *Instruction) DecodeError!bool {
        if (bytes[startIndex] & ADD_REG_RM_MASK != ADD_REG_RM_PATTERN) {
            return false;
        }

        const opcodedw = bytes[startIndex];
        instruction.op = .add;
        instruction.d = if (opcodedw & 0x02 == 0) 0 else 1;
        instruction.w = if (opcodedw & 0x01 == 0) 0 else 1;

        var offset: usize = 0;
        try decodeModRegRm(bytes, startIndex, &offset, instruction);
        instruction.consumedBytes = offset;
        return true;
    }

    fn decodeAddImmToRm(bytes: []const u8, startIndex: usize, instruction: *Instruction) DecodeError!bool {
        if (bytes[startIndex] & ADD_IMM_RM_MASK != ADD_IMM_RM_PATTERN) {
            return false;
        }

        const modregrm = bytes[startIndex + 1];
        const reg_field = @as(u3, @truncate((modregrm & 0x38) >> 3));
        if (reg_field != REG_FIELD_ADD) {
            return false;
        }

        instruction.op = .add;
        instruction.s = if (bytes[startIndex] & 0x02 == 0) 0 else 1;
        instruction.w = if (bytes[startIndex] & 0x01 == 0) 0 else 1;

        var offset: usize = 0;
        try decodeModRegRm(bytes, startIndex, &offset, instruction);
        try decodeImmediate(bytes, startIndex, &offset, instruction);
        instruction.consumedBytes = offset;
        return true;
    }

    fn decodeAddImmToAcc(bytes: []const u8, startIndex: usize, instruction: *Instruction) DecodeError!bool {
        if (bytes[startIndex] & ADD_IMM_ACC_MASK != ADD_IMM_ACC_PATTERN) {
            return false;
        }

        instruction.op = .add;
        instruction.w = if (bytes[startIndex] & 0x01 == 0) 0 else 1;
        instruction.mod = 0b11;
        instruction.reg = 0; // Accumulator
        instruction.d = 1; // Destination is accumulator

        var offset: usize = 1;
        try decodeImmediate(bytes, startIndex, &offset, instruction);
        instruction.consumedBytes = offset;
        return true;
    }

    // SUB instruction decoders
    fn decodeSubRegRm(bytes: []const u8, startIndex: usize, instruction: *Instruction) DecodeError!bool {
        if (bytes[startIndex] & SUB_REG_RM_MASK != SUB_REG_RM_PATTERN) {
            return false;
        }

        const opcodedw = bytes[startIndex];
        instruction.op = .sub;
        instruction.d = if (opcodedw & 0x02 == 0) 0 else 1;
        instruction.w = if (opcodedw & 0x01 == 0) 0 else 1;

        var offset: usize = 0;
        try decodeModRegRm(bytes, startIndex, &offset, instruction);
        instruction.consumedBytes = offset;
        return true;
    }

    fn decodeSubImmToRm(bytes: []const u8, startIndex: usize, instruction: *Instruction) DecodeError!bool {
        if (bytes[startIndex] & ADD_IMM_RM_MASK != ADD_IMM_RM_PATTERN) {
            return false;
        }

        const modregrm = bytes[startIndex + 1];
        const reg_field = @as(u3, @truncate((modregrm & 0x38) >> 3));
        if (reg_field != REG_FIELD_SUB) {
            return false;
        }

        instruction.op = .sub;
        instruction.s = if (bytes[startIndex] & 0x02 == 0) 0 else 1;
        instruction.w = if (bytes[startIndex] & 0x01 == 0) 0 else 1;

        var offset: usize = 0;
        try decodeModRegRm(bytes, startIndex, &offset, instruction);
        try decodeImmediate(bytes, startIndex, &offset, instruction);
        instruction.consumedBytes = offset;
        return true;
    }

    fn decodeSubImmToAcc(bytes: []const u8, startIndex: usize, instruction: *Instruction) DecodeError!bool {
        if (bytes[startIndex] & SUB_IMM_ACC_MASK != SUB_IMM_ACC_PATTERN) {
            return false;
        }

        instruction.op = .sub;
        instruction.w = if (bytes[startIndex] & 0x01 == 0) 0 else 1;
        instruction.mod = 0b11;
        instruction.reg = 0; // Accumulator
        instruction.d = 1; // Destination is accumulator

        var offset: usize = 1;
        try decodeImmediate(bytes, startIndex, &offset, instruction);
        instruction.consumedBytes = offset;
        return true;
    }

    // CMP instruction decoders
    fn decodeCmpRegRm(bytes: []const u8, startIndex: usize, instruction: *Instruction) DecodeError!bool {
        if (bytes[startIndex] & CMP_REG_RM_MASK != CMP_REG_RM_PATTERN) {
            return false;
        }

        const opcodedw = bytes[startIndex];
        instruction.op = .cmp;
        instruction.d = if (opcodedw & 0x02 == 0) 0 else 1;
        instruction.w = if (opcodedw & 0x01 == 0) 0 else 1;

        var offset: usize = 0;
        try decodeModRegRm(bytes, startIndex, &offset, instruction);
        instruction.consumedBytes = offset;
        return true;
    }

    fn decodeCmpImmToRm(bytes: []const u8, startIndex: usize, instruction: *Instruction) DecodeError!bool {
        if (bytes[startIndex] & ADD_IMM_RM_MASK != ADD_IMM_RM_PATTERN) {
            return false;
        }

        const modregrm = bytes[startIndex + 1];
        const reg_field = @as(u3, @truncate((modregrm & 0x38) >> 3));
        if (reg_field != REG_FIELD_CMP) {
            return false;
        }

        instruction.op = .cmp;
        instruction.s = if (bytes[startIndex] & 0x02 == 0) 0 else 1;
        instruction.w = if (bytes[startIndex] & 0x01 == 0) 0 else 1;

        var offset: usize = 0;
        try decodeModRegRm(bytes, startIndex, &offset, instruction);
        try decodeImmediate(bytes, startIndex, &offset, instruction);
        instruction.consumedBytes = offset;
        return true;
    }

    fn decodeCmpImmToAcc(bytes: []const u8, startIndex: usize, instruction: *Instruction) DecodeError!bool {
        if (bytes[startIndex] & CMP_IMM_ACC_MASK != CMP_IMM_ACC_PATTERN) {
            return false;
        }

        instruction.op = .cmp;
        instruction.w = if (bytes[startIndex] & 0x01 == 0) 0 else 1;
        instruction.mod = 0b11;
        instruction.reg = 0; // Accumulator
        instruction.d = 1; // Destination is accumulator

        var offset: usize = 1;
        try decodeImmediate(bytes, startIndex, &offset, instruction);
        instruction.consumedBytes = offset;
        return true;
    }

    fn decodeConditionalJump(bytes: []const u8, startIndex: usize, instruction: *Instruction) DecodeError!bool {
        const opcode = bytes[startIndex];
        instruction.op = switch (opcode) {
            JE_PATTERN => .je,
            JL_PATTERN => .jl,
            JLE_PATTERN => .jle,
            JB_PATTERN => .jb,
            JBE_PATTERN => .jbe,
            JP_PATTERN => .jp,
            JO_PATTERN => .jo,
            JS_PATTERN => .js,
            JNE_PATTERN => .jne,
            JNL_PATTERN => .jnl,
            JG_PATTERN => .jg,
            JNB_PATTERN => .jnb,
            JA_PATTERN => .ja,
            JNP_PATTERN => .jnp,
            JNO_PATTERN => .jno,
            JNS_PATTERN => .jns,
            LOOP_PATTERN => .loop,
            LOOPZ_PATTERN => .loopz,
            LOOPNZ_PATTERN => .loopnz,
            JCXZ_PATTERN => .jcxz,
            else => .invalid,
        };

        if (instruction.op == .invalid) {
            return false;
        }

        if (bytes.len < startIndex + 2) {
            return error.InsufficientBytes;
        }
        instruction.increment = @bitCast(bytes[startIndex + 1]);
        instruction.consumedBytes = 2;
        return true;
    }

    pub fn format(
        self: Instruction,
        comptime fmt: []const u8,
        options: std.fmt.FormatOptions,
        writer: anytype,
    ) !void {
        _ = fmt;
        _ = options;

        try writer.print("{s}", .{@tagName(self.op)});

        switch (self.op) {
            .mov => try self.formatMov(writer),
            .add => try self.formatArithmeticOp(writer),
            .sub => try self.formatArithmeticOp(writer),
            .cmp => try self.formatArithmeticOp(writer),
            .je => try writer.print(" {d}", .{self.increment.?}),
            .jl => try writer.print(" {d}", .{self.increment.?}),
            .jle => try writer.print(" {d}", .{self.increment.?}),
            .jb => try writer.print(" {d}", .{self.increment.?}),
            .jbe => try writer.print(" {d}", .{self.increment.?}),
            .jp => try writer.print(" {d}", .{self.increment.?}),
            .jo => try writer.print(" {d}", .{self.increment.?}),
            .js => try writer.print(" {d}", .{self.increment.?}),
            .jne => try writer.print(" {d}", .{self.increment.?}),
            .jnl => try writer.print(" {d}", .{self.increment.?}),
            .jg => try writer.print(" {d}", .{self.increment.?}),
            .jnb => try writer.print(" {d}", .{self.increment.?}),
            .ja => try writer.print(" {d}", .{self.increment.?}),
            .jnp => try writer.print(" {d}", .{self.increment.?}),
            .jno => try writer.print(" {d}", .{self.increment.?}),
            .jns => try writer.print(" {d}", .{self.increment.?}),
            .loop => try writer.print(" {d}", .{self.increment.?}),
            .loopz => try writer.print(" {d}", .{self.increment.?}),
            .loopnz => try writer.print(" {d}", .{self.increment.?}),
            .jcxz => try writer.print(" {d}", .{self.increment.?}),
            .invalid => {},
        }
    }

    // Common formatter for both MOV and arithmetic operations
    fn formatCommonInstruction(self: Instruction, writer: anytype) !void {
        // Handle accumulator direct address
        if (self.rm == 0b110 and self.mod == 0b00 and self.reg == 0 and (self.immediate == null or self.d == 1)) {
            return if (self.d == 1)
                try writer.print(" {s}, [{d}]", .{
                    registerName(0, self.w),
                    self.displacement.?,
                })
            else
                try writer.print(" [{d}], {s}", .{
                    self.displacement.?,
                    registerName(0, self.w),
                });
        }

        // Handle immediate to register
        if (self.immediate != null and self.mod == 0b11) {
            // For arithmetic operations with immediate operands (ADD/SUB/CMP),
            // the destination register is in the rm field, not reg
            const regField = switch (self.op) {
                .add, .sub, .cmp => self.rm,
                else => self.reg,
            };
            return try writer.print(" {s}, {d}", .{
                registerName(regField, self.w),
                self.immediate.?,
            });
        }

        // Handle register to register
        if (self.mod == 0b11) {
            const dest = if (self.d == 1) self.reg else self.rm;
            const src = if (self.d == 1) self.rm else self.reg;
            return try writer.print(" {s}, {s}", .{
                registerName(dest, self.w),
                registerName(src, self.w),
            });
        }

        // Handle direct address
        if (self.mod == 0b00 and self.rm == 0b110) {
            if (self.immediate != null) {
                return try writer.print(" [{d}], {s} {d}", .{
                    self.displacement.?,
                    if (self.w == 1) "word" else "byte",
                    self.immediate.?,
                });
            }
            return if (self.d == 1)
                try writer.print(" {s}, [{d}]", .{
                    registerName(self.reg, self.w),
                    self.displacement.?,
                })
            else
                try writer.print(" [{d}], {s}", .{
                    self.displacement.?,
                    registerName(self.reg, self.w),
                });
        }

        // Handle memory operations
        const rm_name = getRmName(self.rm);
        if (self.immediate != null) {
            return if (self.displacement) |d|
                try writer.print(" {s}{s}{d}], {s} {d}", .{
                    rm_name,
                    if (d >= 0) " + " else " - ",
                    if (d >= 0) d else -d,
                    if (self.w == 1) "word" else "byte",
                    self.immediate.?,
                })
            else
                try writer.print(" {s}], {s} {d}", .{
                    rm_name,
                    if (self.w == 1) "word" else "byte",
                    self.immediate.?,
                });
        }

        return if (self.d == 1)
            if (self.displacement) |d|
                try writer.print(" {s}, {s}{s}{d}]", .{
                    registerName(self.reg, self.w),
                    rm_name,
                    if (d >= 0) " + " else " - ",
                    if (d >= 0) d else -d,
                })
            else
                try writer.print(" {s}, {s}]", .{
                    registerName(self.reg, self.w),
                    rm_name,
                })
        else if (self.displacement) |d|
            try writer.print(" {s}{s}{d}], {s}", .{
                rm_name,
                if (d >= 0) " + " else " - ",
                if (d >= 0) d else -d,
                registerName(self.reg, self.w),
            })
        else
            try writer.print(" {s}], {s}", .{
                rm_name,
                registerName(self.reg, self.w),
            });
    }

    fn formatMov(self: Instruction, writer: anytype) !void {
        try self.formatCommonInstruction(writer);
    }

    fn formatArithmeticOp(self: Instruction, writer: anytype) !void {
        try self.formatCommonInstruction(writer);
    }
};

pub fn decodeInstructions(binaryInstructions: []const u8, allocator: Allocator) !ArrayList(Instruction) {
    var decodedInstructions = ArrayList(Instruction).init(allocator);

    var i: usize = 0;
    while (i < binaryInstructions.len) {
        const instruction = try Instruction.decode(binaryInstructions, i);
        if (instruction.op == .invalid) {
            return error.InvalidInstruction;
        }
        try decodedInstructions.append(instruction);
        i += instruction.consumedBytes;
    }
    return decodedInstructions;
}

pub fn formatInstructions(instructions: []const Instruction, writer: anytype) !void {
    try writer.writeAll("bits 16\n");
    for (instructions) |instruction| {
        try instruction.format("", .{}, writer);
        try writer.writeByte('\n');
    }
}

test "MOV reg, reg/mem pattern single instruction" {
    var buffer = [_]u8{ 0b10001001, 0b11011001 };
    const expected =
        \\bits 16
        \\mov cx, bx
        \\
    ;
    const decodedInstructions = try decodeInstructions(
        buffer[0..buffer.len],
        testing.allocator,
    );
    defer decodedInstructions.deinit();

    var actual = ArrayList(u8).init(testing.allocator);
    defer actual.deinit();
    try formatInstructions(decodedInstructions.items, actual.writer());

    try testing.expectEqualStrings(expected, actual.items);
}

test "MOV reg, reg/mem pattern multiple instructions" {
    var buffer = [_]u8{ 0b10001001, 0b11011001, 0b10001000, 0b11100101, 0b10001001, 0b11011010, 0b10001001, 0b11011110, 0b10001001, 0b11111011, 0b10001000, 0b11001000, 0b10001000, 0b11101101, 0b10001001, 0b11000011, 0b10001001, 0b11110011, 0b10001001, 0b11111100, 0b10001001, 0b11000101 };
    const expected =
        \\bits 16
        \\mov cx, bx
        \\mov ch, ah
        \\mov dx, bx
        \\mov si, bx
        \\mov bx, di
        \\mov al, cl
        \\mov ch, ch
        \\mov bx, ax
        \\mov bx, si
        \\mov sp, di
        \\mov bp, ax
        \\
    ;

    const decodedInstructions = try decodeInstructions(buffer[0..buffer.len], testing.allocator);
    defer decodedInstructions.deinit();

    var actual = ArrayList(u8).init(testing.allocator);
    defer actual.deinit();
    try formatInstructions(decodedInstructions.items, actual.writer());

    try testing.expectEqualStrings(expected, actual.items);
}

test "More MOVs (including immediate to register)" {
    var buffer = [_]u8{ 0b10001001, 0b11011110, 0b10001000, 0b11000110, 0b10110001, 0b00001100, 0b10110101, 0b11110100, 0b10111001, 0b00001100, 0b00000000, 0b10111001, 0b11110100, 0b11111111, 0b10111010, 0b01101100, 0b00001111, 0b10111010, 0b10010100, 0b11110000, 0b10001010, 0b00000000, 0b10001011, 0b00011011, 0b10001011, 0b01010110, 0b00000000, 0b10001010, 0b01100000, 0b00000100, 0b10001010, 0b10000000, 0b10000111, 0b00010011, 0b10001001, 0b00001001, 0b10001000, 0b00001010, 0b10001000, 0b01101110, 0b00000000 };
    const expected =
        \\bits 16
        \\mov si, bx
        \\mov dh, al
        \\mov cl, 12
        \\mov ch, -12
        \\mov cx, 12
        \\mov cx, -12
        \\mov dx, 3948
        \\mov dx, -3948
        \\mov al, [bx + si]
        \\mov bx, [bp + di]
        \\mov dx, [bp + 0]
        \\mov ah, [bx + si + 4]
        \\mov al, [bx + si + 4999]
        \\mov [bx + di], cx
        \\mov [bp + si], cl
        \\mov [bp + 0], ch
        \\
    ;

    const decodedInstructions = try decodeInstructions(buffer[0..buffer.len], testing.allocator);
    defer decodedInstructions.deinit();

    var actual = ArrayList(u8).init(testing.allocator);
    defer actual.deinit();
    try formatInstructions(decodedInstructions.items, actual.writer());

    try testing.expectEqualStrings(expected, actual.items);
}

test "Challenge MOVs" {
    var buffer = [_]u8{ 0b10001011, 0b01000001, 0b11011011, 0b10001001, 0b10001100, 0b11010100, 0b11111110, 0b10001011, 0b01010111, 0b11100000, 0b11000110, 0b00000011, 0b00000111, 0b11000111, 0b10000101, 0b10000101, 0b00000011, 0b01011011, 0b00000001, 0b10001011, 0b00101110, 0b00000101, 0b00000000, 0b10001011, 0b00011110, 0b10000010, 0b00001101, 0b10100001, 0b11111011, 0b00001001, 0b10100001, 0b00010000, 0b00000000, 0b10100011, 0b11111010, 0b00001001, 0b10100011, 0b00001111, 0b00000000 };
    const expected =
        \\bits 16
        \\mov ax, [bx + di - 37]
        \\mov [si - 300], cx
        \\mov dx, [bx - 32]
        \\mov [bp + di], byte 7
        \\mov [di + 901], word 347
        \\mov bp, [5]
        \\mov bx, [3458]
        \\mov ax, [2555]
        \\mov ax, [16]
        \\mov [2554], ax
        \\mov [15], ax
        \\
    ;

    const decodedInstructions = try decodeInstructions(buffer[0..buffer.len], testing.allocator);
    defer decodedInstructions.deinit();

    var actual = ArrayList(u8).init(testing.allocator);
    defer actual.deinit();
    try formatInstructions(decodedInstructions.items, actual.writer());

    try testing.expectEqualStrings(expected, actual.items);
}

test "ADD instructions" {
    // Testing ADD with different addressing modes
    var buffer = [_]u8{
        0b00000011, 0b11000011,
        0b00000011, 0b00001000,
        0b00000001, 0b00010011,
        0b10000001, 0b00000111,
        0b00000101, 0b00000000,
        0b00000100, 0b00101010,
    };

    const expected =
        \\bits 16
        \\add ax, bx
        \\add cx, [bx + si]
        \\add [bp + di], dx
        \\add [bx], word 5
        \\add al, 42
        \\
    ;

    const decodedInstructions = try decodeInstructions(buffer[0..buffer.len], testing.allocator);
    defer decodedInstructions.deinit();

    var actual = ArrayList(u8).init(testing.allocator);
    defer actual.deinit();
    try formatInstructions(decodedInstructions.items, actual.writer());

    try testing.expectEqualStrings(expected, actual.items);
}

test "SUB instructions" {
    // Testing SUB with different addressing modes
    var buffer = [_]u8{
        0b00101011, 0b11000011,
        0b00101011, 0b00001000,
        0b00101001, 0b00010011,
        0b10000001, 0b00101111,
        0b00000101, 0b00000000,
        0b00101100, 0b00101010,
    };

    const expected =
        \\bits 16
        \\sub ax, bx
        \\sub cx, [bx + si]
        \\sub [bp + di], dx
        \\sub [bx], word 5
        \\sub al, 42
        \\
    ;

    const decodedInstructions = try decodeInstructions(buffer[0..buffer.len], testing.allocator);
    defer decodedInstructions.deinit();

    var actual = ArrayList(u8).init(testing.allocator);
    defer actual.deinit();
    try formatInstructions(decodedInstructions.items, actual.writer());

    try testing.expectEqualStrings(expected, actual.items);
}

test "CMP instructions" {
    // Testing CMP with different addressing modes
    var buffer = [_]u8{
        0b00111011, 0b11000011,
        0b00111011, 0b00001000,
        0b00111001, 0b00010011,
        0b10000001, 0b00111111,
        0b00000101, 0b00000000,
        0b00111100, 0b00101010,
    };

    const expected =
        \\bits 16
        \\cmp ax, bx
        \\cmp cx, [bx + si]
        \\cmp [bp + di], dx
        \\cmp [bx], word 5
        \\cmp al, 42
        \\
    ;

    const decodedInstructions = try decodeInstructions(buffer[0..buffer.len], testing.allocator);
    defer decodedInstructions.deinit();

    var actual = ArrayList(u8).init(testing.allocator);
    defer actual.deinit();
    try formatInstructions(decodedInstructions.items, actual.writer());

    try testing.expectEqualStrings(expected, actual.items);
}
