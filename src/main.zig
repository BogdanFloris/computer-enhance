const std = @import("std");
const decoder = @import("decoder.zig");
const e = @import("executor.zig");

var gpa = std.heap.GeneralPurposeAllocator(.{ .thread_safe = true }){};

pub fn main() !void {
    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer arena.deinit();

    const allocator = arena.allocator();

    const file = std.fs.cwd().openFile("asm/immediate_movs", .{ .mode = .read_only }) catch |err| {
        std.debug.print("Failed to open file: {s}\n", .{@errorName(err)});
        return;
    };
    defer file.close();

    const fileSize = (try file.stat()).size;
    const buffer = try allocator.alloc(u8, fileSize);
    _ = try file.read(buffer);
    // printBytes(buffer);

    const decodedInstructions = try decoder.decodeInstructions(buffer, allocator);
    defer decodedInstructions.deinit();

    const writer = std.io.getStdOut().writer();
    try decoder.formatInstructions(decodedInstructions.items, writer);
    try writer.print("\n", .{});

    var executor = e.Executor.new();
    try executor.executeInstructions(decodedInstructions.items);
    try executor.printRegisters("", .{}, writer);
}

fn printBytes(bytes: []const u8) void {
    for (bytes) |byte| {
        std.debug.print("0x{X} ", .{byte});
    }
    std.debug.print("\n", .{});
}
