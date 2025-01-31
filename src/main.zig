const std = @import("std");
const decoder = @import("decoder.zig");

var gpa = std.heap.GeneralPurposeAllocator(.{ .thread_safe = true }){};

pub fn main() !void {
    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer arena.deinit();

    const allocator = arena.allocator();

    const file = std.fs.cwd().openFile("asm/more_movs", .{ .mode = .read_only }) catch |err| {
        std.debug.print("Failed to open file: {s}\n", .{@errorName(err)});
        return;
    };
    defer file.close();

    const fileSize = (try file.stat()).size;
    const buffer = try allocator.alloc(u8, fileSize);
    _ = try file.read(buffer);

    const decodedInstructions = try decoder.decodeInstructions(buffer, allocator);
    defer decodedInstructions.deinit();

    const writer = std.io.getStdOut().writer();
    try decoder.formatInstructions(decodedInstructions.items, writer);
}
