const std = @import("std");
const Instruction = @import("decoder.zig").Instruction;

pub const Executor = struct {
    // registers
    ax: u16, // al (u8), ah (u8)
    bx: u16, // bl (u8), bh (u8)
    cx: u16, // cl (u8), ch (u8)
    dx: u16, // dl (u8), dh (u8)
    sp: u16,
    bp: u16,
    si: u16,
    di: u16,

    pub fn new() Executor {
        return Executor{ .ax = 0, .bx = 0, .cx = 0, .dx = 0, .sp = 0, .bp = 0, .si = 0, .di = 0 };
    }

    pub fn getRegister(self: *Executor, reg: u3, w: u1) *u16 {
        return switch (w) {
            0 => switch (reg) {
                0b000, 0b100 => &self.ax,
                0b001, 0b101 => &self.cx,
                0b010, 0b110 => &self.dx,
                0b011, 0b111 => &self.bx,
            },
            1 => switch (reg) {
                0b000 => &self.ax,
                0b001 => &self.cx,
                0b010 => &self.dx,
                0b011 => &self.bx,
                0b100 => &self.sp,
                0b101 => &self.bp,
                0b110 => &self.si,
                0b111 => &self.di,
            },
        };
    }

    pub fn executeInstructions(self: *Executor, instructions: []const Instruction) !void {
        for (instructions) |instruction| {
            switch (instruction.op) {
                .mov => {
                    try self.executeMov(instruction);
                },
                .add => {},
                .sub => {},
                .cmp => {},
                .je, .jl, .jle, .jb, .jbe, .jp, .jo, .js, .jne, .jnl, .jg, .jnb, .ja, .jnp, .jno, .jns, .loop, .loopz, .loopnz, .jcxz => {},
                .invalid => {},
            }
        }
    }

    fn executeMov(self: *Executor, instruction: Instruction) !void {
        const reg = self.getRegister(instruction.reg, instruction.w);
        reg.* = @bitCast(instruction.immediate.?);
    }

    pub fn printRegisters(self: Executor, comptime fmt: []const u8, options: std.fmt.FormatOptions, writer: anytype) !void {
        _ = fmt;
        _ = options;

        try writer.print("Final registers:\n", .{});
        try writer.print("    ax: 0x{x:0>4} ({d})\n", .{ self.ax, self.ax });
        try writer.print("    bx: 0x{x:0>4} ({d})\n", .{ self.bx, self.bx });
        try writer.print("    cx: 0x{x:0>4} ({d})\n", .{ self.cx, self.cx });
        try writer.print("    dx: 0x{x:0>4} ({d})\n", .{ self.dx, self.dx });
        try writer.print("    sp: 0x{x:0>4} ({d})\n", .{ self.sp, self.sp });
        try writer.print("    bp: 0x{x:0>4} ({d})\n", .{ self.bp, self.bp });
        try writer.print("    si: 0x{x:0>4} ({d})\n", .{ self.si, self.si });
        try writer.print("    di: 0x{x:0>4} ({d})\n", .{ self.di, self.di });
    }
};
