const std = @import("std");
const c = @cImport({
    @cInclude("raylib.h");
    @cInclude("raymath.h");
    @cInclude("joltc.h");
});

const WINDOW_WIDTH = 800;
const WINDOW_HEIGHT = 600;
const DELTA_TIME = 1.0 / 60.0;

const Ball = struct {
    body_id: c.JPH_BodyID,
    selected: bool,
};

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    const allocator = gpa.allocator();
    defer {
        const check = gpa.deinit();
        switch (check) {
            .leak => std.debug.print("Memory leak(s) detected!\n", .{}),
            .ok => std.debug.print("All memory freed\n", .{}),
        }
    }

    var prng = std.Random.DefaultPrng.init(42069);
    var rand = prng.random();

    c.InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Samples");
    defer c.CloseWindow();
    c.SetTargetFPS(60);

    var camera = c.Camera3D{
        .position = .{ .x = 0, .y = 5, .z = 5 },
        .target = .{ .x = 0, .y = 0, .z = 0 },
        .up = .{ .x = 0, .y = 1, .z = 0 },
        .fovy = 45,
        .projection = c.CAMERA_PERSPECTIVE,
    };

    const sphere_mesh = c.GenMeshSphere(0.5, 8, 16);
    var sphere = c.LoadModelFromMesh(sphere_mesh);
    defer c.UnloadModel(sphere);

    const OBJECT_LAYER_NON_MOVING = 0;
    const OBJECT_LAYER_MOVING = 1;
    const OBJECT_LAYER_NUM = 2;

    const BROAD_PHASE_LAYER_NON_MOVING = 0;
    const BROAD_PHASE_LAYER_MOVING = 1;
    const BROAD_PHASE_LAYER_NUM = 2;

    const ok = c.JPH_Init();
    if (!ok) {
        @panic("Failed to init JPH");
    }
    defer c.JPH_Shutdown();

    const job_system = c.JPH_JobSystemThreadPool_Create(&.{}).?;
    defer c.JPH_JobSystem_Destroy(job_system);

    const object_layer_pair_filter = c.JPH_ObjectLayerPairFilterTable_Create(OBJECT_LAYER_NUM).?;
    c.JPH_ObjectLayerPairFilterTable_EnableCollision(
        object_layer_pair_filter,
        OBJECT_LAYER_MOVING,
        OBJECT_LAYER_MOVING,
    );
    c.JPH_ObjectLayerPairFilterTable_EnableCollision(
        object_layer_pair_filter,
        OBJECT_LAYER_MOVING,
        OBJECT_LAYER_NON_MOVING,
    );

    const broad_phase_layer_interface_table = c.JPH_BroadPhaseLayerInterfaceTable_Create(
        OBJECT_LAYER_NUM,
        BROAD_PHASE_LAYER_NUM,
    ).?;
    c.JPH_BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(
        broad_phase_layer_interface_table,
        OBJECT_LAYER_NON_MOVING,
        BROAD_PHASE_LAYER_NON_MOVING,
    );
    c.JPH_BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(
        broad_phase_layer_interface_table,
        OBJECT_LAYER_MOVING,
        BROAD_PHASE_LAYER_MOVING,
    );

    const object_vs_broad_phase_layer_filter = c.JPH_ObjectVsBroadPhaseLayerFilterTable_Create(
        broad_phase_layer_interface_table,
        BROAD_PHASE_LAYER_NUM,
        object_layer_pair_filter,
        OBJECT_LAYER_NUM,
    ).?;

    const physics_system_settings = c.JPH_PhysicsSystemSettings{
        .maxBodies = 65535,
        .numBodyMutexes = 0,
        .maxBodyPairs = 65535,
        .maxContactConstraints = 65535,
        .broadPhaseLayerInterface = broad_phase_layer_interface_table,
        .objectLayerPairFilter = object_layer_pair_filter,
        .objectVsBroadPhaseLayerFilter = object_vs_broad_phase_layer_filter,
    };
    const physics_system = c.JPH_PhysicsSystem_Create(&physics_system_settings).?;
    defer c.JPH_PhysicsSystem_Destroy(physics_system);

    const body_interface = c.JPH_PhysicsSystem_GetBodyInterface(physics_system).?;

    const floor_id = blk: {
        const shape = c.JPH_BoxShape_Create(&.{ .x = 25.0, .y = 0.5, .z = 25.0 }, c.JPH_DEFAULT_CONVEX_RADIUS).?;
        const settings = c.JPH_BodyCreationSettings_Create3(
            @ptrCast(shape),
            &.{ .x = 0, .y = -0.5, .z = 0 },
            &.{ .w = 1, .y = 0, .x = 0, .z = 0 },
            c.JPH_MotionType_Static,
            OBJECT_LAYER_NON_MOVING,
        );
        defer c.JPH_BodyCreationSettings_Destroy(settings);
        const body_id = c.JPH_BodyInterface_CreateAndAddBody(body_interface, settings, c.JPH_Activation_DontActivate);
        break :blk body_id;
    };
    defer c.JPH_BodyInterface_RemoveAndDestroyBody(body_interface, floor_id);

    const sphere_shape = c.JPH_SphereShape_Create(0.5).?;
    var balls = std.AutoHashMap(c.JPH_BodyID, Ball).init(allocator);
    defer {
        var itr = balls.iterator();
        while (itr.next()) |ball| {
            c.JPH_BodyInterface_RemoveAndDestroyBody(body_interface, ball.value_ptr.body_id);
        }
        balls.deinit();
    }

    var is_spawning = false;

    var removed_balls = std.ArrayList(c.JPH_BodyID).init(allocator);
    defer removed_balls.deinit();

    const narrow_phase_query = c.JPH_PhysicsSystem_GetNarrowPhaseQuery(physics_system);

    var ray: ?c.Ray = null;
    while (!c.WindowShouldClose()) {
        const err = c.JPH_PhysicsSystem_Update(physics_system, DELTA_TIME, 1, job_system);
        if (err != c.JPH_PhysicsUpdateError_None) {
            @panic("Failed to update JPH");
        }

        if (c.IsMouseButtonDown(c.MOUSE_BUTTON_RIGHT)) {
            c.UpdateCamera(&camera, c.CAMERA_FREE);
        } else {
            if (c.IsKeyPressed(c.KEY_SPACE)) {
                is_spawning = !is_spawning;
            }
        }

        if (c.IsMouseButtonPressed(c.MOUSE_BUTTON_LEFT)) {
            ray = c.GetScreenToWorldRay(c.GetMousePosition(), camera);
            var result: c.JPH_RayCastResult = undefined;
            if (ray) |r| {
                if (c.JPH_NarrowPhaseQuery_CastRay(narrow_phase_query, &asVec3(r.position), &asVec3(c.Vector3Scale(r.direction, 100)), &result, null, null, null)) {
                    const hit_body_id = result.bodyID;
                    if (balls.getPtr(hit_body_id)) |body| {
                        body.selected = !body.selected;
                    }
                }
            }
        }

        if (is_spawning) {
            const pos: c.JPH_Vec3 = .{
                .x = @floatFromInt(rand.intRangeAtMost(i32, -5, 5)),
                .y = @floatFromInt(rand.intRangeAtMost(i32, 5, 10)),
                .z = @floatFromInt(rand.intRangeAtMost(i32, -5, 5)),
            };

            const settings = c.JPH_BodyCreationSettings_Create3(
                @ptrCast(sphere_shape),
                &pos,
                &.{ .w = 1, .x = 0, .y = 0, .z = 0 },
                c.JPH_MotionType_Dynamic,
                OBJECT_LAYER_MOVING,
            );
            defer c.JPH_BodyCreationSettings_Destroy(settings);
            const sphere_id = c.JPH_BodyInterface_CreateAndAddBody(body_interface, settings, c.JPH_Activation_Activate);
            c.JPH_BodyInterface_SetRestitution(body_interface, sphere_id, 0.8);

            const ball = Ball{
                .body_id = sphere_id,
                .selected = false,
            };
            try balls.put(sphere_id, ball);
        }

        c.BeginDrawing();
        defer c.EndDrawing();

        c.ClearBackground(c.BLACK);
        c.BeginMode3D(camera);
        {
            var itr = balls.iterator();
            sphere_loop: while (itr.next()) |ball| {
                const ball_val = ball.value_ptr;
                const body_id = ball_val.body_id;
                var position: c.JPH_Vec3 = undefined;
                var rotation: c.JPH_Quat = undefined;

                c.JPH_BodyInterface_GetPosition(body_interface, body_id, &position);
                c.JPH_BodyInterface_GetRotation(body_interface, body_id, &rotation);

                if (position.y <= -50) {
                    c.JPH_BodyInterface_RemoveAndDestroyBody(body_interface, body_id);
                    try removed_balls.append(ball.value_ptr.body_id);
                    continue :sphere_loop;
                }

                sphere.transform = c.QuaternionToMatrix(asQuaternion(rotation));

                const col = if (ball_val.selected) c.WHITE else c.BLACK;

                c.DrawModel(sphere, asVector3(position), 1, c.RED);
                c.DrawModelWires(sphere, asVector3(position), 1, col);
            }

            for (removed_balls.items) |id| {
                _ = balls.remove(id);
            }
            removed_balls.clearAndFree();

            c.DrawGrid(50, 1);
        }
        c.EndMode3D();
        c.DrawFPS(0, 0);

        const text = try std.fmt.allocPrint(
            allocator,
            "Hold Right Mouse Button and WASD for camera controls.\nPress Space to {s} spawning.\nCount: {d}\n",
            .{
                if (is_spawning) "stop" else "start",
                balls.count(),
            },
        );
        defer allocator.free(text);
        c.DrawText(
            text.ptr,
            0,
            20,
            20,
            c.WHITE,
        );
    }
}

fn asVector3(v: c.JPH_Vec3) c.Vector3 {
    return .{
        .x = v.x,
        .y = v.y,
        .z = v.z,
    };
}

fn asVec3(v: c.Vector3) c.JPH_Vec3 {
    return .{
        .x = v.x,
        .y = v.y,
        .z = v.z,
    };
}

fn asMatrix(m: c.JPH_Matrix4x4) c.Matrix {
    return .{
        .m0 = m.m11,
        .m1 = m.m12,
        .m2 = m.m13,
        .m3 = m.m14,
        .m4 = m.m21,
        .m5 = m.m22,
        .m6 = m.m23,
        .m7 = m.m24,
        .m8 = m.m31,
        .m9 = m.m32,
        .m10 = m.m33,
        .m11 = m.m34,
        .m12 = m.m41,
        .m13 = m.m42,
        .m14 = m.m43,
        .m15 = m.m44,
    };
}

fn asQuaternion(q: c.JPH_Quat) c.Quaternion {
    return .{
        .x = q.x,
        .y = q.y,
        .z = q.z,
        .w = q.w,
    };
}
