// 8086 instruction decoder
const std = @import("std");
const testing = std.testing;
const mem = std.mem;
const assert = std.debug.assert;
const Allocator = mem.Allocator;
const ArrayList = std.ArrayList;

// Constants for instruction patterns
const MOV_REG_RM_PATTERN = 0x88;
const MOV_IMM_REG_PATTERN = 0xB0;
const MOV_REG_RM_MASK = 0xFC;
const MOV_IMM_REG_MASK = 0xF0;

// Instruction set components
const Opcode = enum { mov, invalid };

const Instruction = struct {
    op: Opcode,
    d: u1, // direction  (0 - reg to rm, 1 - rm to reg)
    w: u1, // word/byte operation (0 - byte, 1 - word)
    mod: u2, // mode
    reg: u3, // register
    rm: u3, // register/memory
    displacement: ?i16, // displacement can be signed
    immediate: ?i16, // immediate can be signed
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
            .consumedBytes = 0,
        };

        if (try decodeMovRegRm(bytes, startIndex, &instruction)) {
            return instruction;
        }
        if (try decodeMovImmediateToReg(bytes, startIndex, &instruction)) {
            return instruction;
        }

        return instruction;
    }

    const DecodeError = error{
        InsufficientBytes,
        InvalidOpcode,
    };

    fn decodeMovRegRm(bytes: []const u8, startIndex: usize, instruction: *Instruction) DecodeError!bool {
        if (bytes[startIndex] & MOV_REG_RM_MASK != MOV_REG_RM_PATTERN) {
            return false;
        }

        const opcodedw = bytes[startIndex];
        instruction.op = .mov;
        instruction.d = if (opcodedw & 0x02 == 0) 0 else 1;
        instruction.w = if (opcodedw & 0x01 == 0) 0 else 1;

        const modregrm = bytes[startIndex + 1];
        instruction.mod = @truncate(modregrm >> 6);
        instruction.reg = @truncate((modregrm & 0x38) >> 3);
        instruction.rm = @truncate(modregrm & 0x07);

        // Handle displacement based on mod
        switch (instruction.mod) {
            0b00 => {
                if (instruction.rm == 0b110) {
                    if (bytes.len < startIndex + 4) {
                        return error.InsufficientBytes;
                    }
                    // direct address 16-bit displacement
                    instruction.displacement = readSigned16(bytes, startIndex + 2);
                    instruction.consumedBytes = 4;
                    return true;
                }
                instruction.consumedBytes = 2;
            },
            0b01 => {
                if (bytes.len < startIndex + 3) {
                    return error.InsufficientBytes;
                }
                // 8-bit displacement with sign extension
                instruction.displacement = signExtend8to16(bytes[startIndex + 2]);
                instruction.consumedBytes = 3;
            },
            0b10 => {
                if (bytes.len < startIndex + 4) {
                    return error.InsufficientBytes;
                }
                // 16-bit displacement
                instruction.displacement = readSigned16(bytes, startIndex + 2);
                instruction.consumedBytes = 4;
            },
            0b11 => {
                // no displacement
                instruction.consumedBytes = 2;
            },
        }
        return true;
    }

    fn decodeMovImmediateToReg(bytes: []const u8, startIndex: usize, instruction: *Instruction) DecodeError!bool {
        if (bytes[startIndex] & MOV_IMM_REG_MASK != MOV_IMM_REG_PATTERN) {
            return false;
        }

        instruction.op = .mov;
        instruction.d = 1;
        const opcodewreg = bytes[startIndex];
        instruction.w = if (opcodewreg & 0x08 == 0) 0 else 1;
        instruction.reg = @truncate(opcodewreg & 0x07);

        if (instruction.w == 1) {
            if (bytes.len < startIndex + 3) {
                return error.InsufficientBytes;
            }
            // 16-bit immediate
            instruction.immediate = readSigned16(bytes, startIndex + 1);
            instruction.consumedBytes = 3;
        } else {
            if (bytes.len < startIndex + 2) {
                return error.InsufficientBytes;
            }
            // 8-bit immediate with sign extension
            instruction.immediate = signExtend8to16(bytes[startIndex + 1]);
            instruction.consumedBytes = 2;
        }
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
            .invalid => {},
        }
    }

    fn formatMov(self: Instruction, writer: anytype) !void {
        const dest = if (self.d == 1) self.reg else self.rm;
        const src = if (self.d == 1) self.rm else self.reg;

        if (self.mod == 0b11) {
            // Register to register
            try writer.print(" {s}, {s}", .{
                registerName(dest, self.w),
                registerName(src, self.w),
            });
            return;
        }

        // Handle immediate to register
        if (self.immediate != null) {
            try writer.print(" {s}, {d}", .{
                registerName(dest, self.w),
                self.immediate.?,
            });
            return;
        }

        // Handle direct address (mod 00, r/m 110)
        if (self.mod == 0b00 and self.rm == 0b110) {
            if (self.d == 1) {
                try writer.print(" {s}, [{d}]", .{
                    registerName(dest, self.w),
                    self.displacement.?,
                });
            } else {
                try writer.print(" [{d}], {s}", .{
                    self.displacement.?,
                    registerName(src, self.w),
                });
            }
            return;
        }

        // Handle other memory addressing modes
        const rm_name = getRmName(self.rm);
        if (self.d == 1) {
            if (self.displacement != null) {
                if (self.displacement.? >= 0) {
                    try writer.print(" {s}, {s} + {d}]", .{
                        registerName(dest, self.w),
                        rm_name,
                        self.displacement.?,
                    });
                } else {
                    try writer.print(" {s}, {s} - {d}]", .{
                        registerName(dest, self.w),
                        rm_name,
                        -self.displacement.?,
                    });
                }
            } else {
                try writer.print(" {s}, {s}]", .{
                    registerName(dest, self.w),
                    rm_name,
                });
            }
        } else {
            if (self.displacement != null) {
                if (self.displacement.? >= 0) {
                    try writer.print(" {s} + {d}], {s}", .{
                        rm_name,
                        self.displacement.?,
                        registerName(src, self.w),
                    });
                } else {
                    try writer.print(" {s} - {d}], {s}", .{
                        rm_name,
                        -self.displacement.?,
                        registerName(src, self.w),
                    });
                }
            } else {
                try writer.print(" {s}], {s}", .{
                    rm_name,
                    registerName(src, self.w),
                });
            }
        }
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
