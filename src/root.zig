const std = @import("std");
const testing = std.testing;
const mem = std.mem;
const assert = std.debug.assert;
const Allocator = mem.Allocator;
const ArrayList = std.ArrayList;

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

const Opcode = enum { mov, invalid };

const Instruction = struct {
    op: Opcode,
    d: u1, // direction  (0 - reg to rm, 1 - rm to reg)
    w: u1, // word/byte operation (0 - byte, 1 - word)
    mod: u2, // mode
    reg: u3, // register
    rm: u3, // register/memory

    pub fn decode(bytes: []const u8) !Instruction {
        assert(bytes.len == 2);
        var instruction = Instruction{ .op = .invalid, .d = 0, .w = 0, .mod = 0b00, .reg = 0b000, .rm = 0b000 };

        // MOV reg, reg/mem pattern (100010dw)
        if (bytes[0] & 0xFC == 0x88) {
            instruction.op = .mov;
            instruction.d = if (bytes[0] & 0x02 == 0) 0 else 1;
            instruction.w = if (bytes[0] & 0x01 == 0) 0 else 1;

            const modregrm = bytes[1];
            instruction.mod = @truncate(modregrm >> 6);
            instruction.reg = @truncate((modregrm & 0x38) >> 3);
            instruction.rm = @truncate(modregrm & 0x07);
        }

        return instruction;
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
            .mov => {
                const dest = if (self.d == 1) self.reg else self.rm;
                const src = if (self.d == 1) self.rm else self.reg;
                try writer.print(" {s}, {s}", .{
                    registerName(dest, self.w),
                    registerName(src, self.w),
                });
            },
            .invalid => {},
        }
    }
};

pub fn decodeInstructions(binaryInstructions: []const u8, allocator: Allocator) ![]Instruction {
    if (binaryInstructions.len % 2 != 0) {
        // Each instruction is 16 bits,
        // so we cannot have an odd number of elements in the array
        return error.InvalidBinaryLength;
    }

    const decodedInstructions: []Instruction = try allocator.alloc(Instruction, binaryInstructions.len / 2);

    var i: usize = 0;
    while (i < binaryInstructions.len) {
        const instruction = try Instruction.decode(binaryInstructions[i .. i + 2]);
        decodedInstructions[i / 2] = instruction;
        i += 2;
    }
    return decodedInstructions;
}

pub fn formatInstructions(instructions: []const Instruction, writer: anytype) !void {
    try writer.writeAll("bits 16\n\n");
    for (instructions) |instruction| {
        try instruction.format("", .{}, writer);
        try writer.writeByte('\n');
    }
}

test "MOV reg, reg/mem pattern single instruction" {
    var buffer = [_]u8{ 0b10001001, 0b11011001 };
    const expected =
        \\bits 16
        \\
        \\mov cx, bx
        \\
    ;
    const decodedInstructions = try decodeInstructions(buffer[0..buffer.len], testing.allocator);
    defer testing.allocator.free(decodedInstructions);

    var actual = ArrayList(u8).init(testing.allocator);
    defer actual.deinit();
    try formatInstructions(decodedInstructions, actual.writer());

    try testing.expectEqualStrings(expected, actual.items);
}

test "MOV reg, reg/mem pattern multiple instructions" {
    var buffer = [_]u8{ 0b10001001, 0b11011001, 0b10001000, 0b11100101, 0b10001001, 0b11011010, 0b10001001, 0b11011110, 0b10001001, 0b11111011, 0b10001000, 0b11001000, 0b10001000, 0b11101101, 0b10001001, 0b11000011, 0b10001001, 0b11110011, 0b10001001, 0b11111100, 0b10001001, 0b11000101 };
    const expected =
        \\bits 16
        \\
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
    defer testing.allocator.free(decodedInstructions);

    var actual = ArrayList(u8).init(testing.allocator);
    defer actual.deinit();
    try formatInstructions(decodedInstructions, actual.writer());

    try testing.expectEqualStrings(expected, actual.items);
}

test "invalid binary length" {
    var buffer = [_]u8{0b10001001};
    const decodedInstructions = decodeInstructions(buffer[0..buffer.len], testing.allocator);
    try testing.expectError(error.InvalidBinaryLength, decodedInstructions);
}
