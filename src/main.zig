const std = @import("std");
const ce = @import("root.zig");

var gpa = std.heap.GeneralPurposeAllocator(.{ .thread_safe = true }){};

pub fn main() !void {
    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer arena.deinit();

    const allocator = arena.allocator();

    const file = std.fs.cwd().openFile("asm/single_register_mov", .{ .mode = .read_only }) catch |err| {
        std.debug.print("Failed to open file: {s}\n", .{@errorName(err)});
        return;
    };
    defer file.close();

    const fileSize = (try file.stat()).size;
    const buffer = try allocator.alloc(u8, fileSize);
    _ = try file.read(buffer);

    const asmInstructions = try ce.decodeInstructions(buffer, allocator);
    defer allocator.free(asmInstructions);

    const writer = std.io.getStdOut().writer();
    try ce.formatInstructions(asmInstructions, writer);
}
