const c = @cImport({
    @cInclude("raylib.h");
    @cInclude("raymath.h");
    @cInclude("joltc.h");
});

const WINDOW_WIDTH = 800;
const WINDOW_HEIGHT = 600;
const DELTA_TIME = 1.0 / 60.0;

pub fn main() !void {
    c.InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Ballpit");
    defer c.CloseWindow();
    c.SetTargetFPS(60);

    var camera = c.Camera3D{
        .position = .{ .x = 0, .y = 5, .z = 5 },
        .target = .{ .x = 0, .y = 0, .z = 0 },
        .up = .{ .x = 0, .y = 1, .z = 0 },
        .fovy = 45,
        .projection = c.CAMERA_PERSPECTIVE,
    };

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
        .maxBodies = 1024,
        .numBodyMutexes = 0,
        .maxBodyPairs = 1024,
        .maxContactConstraints = 1024,
        .broadPhaseLayerInterface = broad_phase_layer_interface_table,
        .objectLayerPairFilter = object_layer_pair_filter,
        .objectVsBroadPhaseLayerFilter = object_vs_broad_phase_layer_filter,
    };
    const physics_system = c.JPH_PhysicsSystem_Create(&physics_system_settings).?;
    defer c.JPH_PhysicsSystem_Destroy(physics_system);

    const body_interface = c.JPH_PhysicsSystem_GetBodyInterface(physics_system).?;

    const floor_id = blk: {
        const shape = c.JPH_BoxShape_Create(&.{ .x = 100.0, .y = 1.0, .z = 100.0 }, c.JPH_DEFAULT_CONVEX_RADIUS).?;
        const settings = c.JPH_BodyCreationSettings_Create3(
            @ptrCast(shape),
            &.{ .x = 0, .y = -1, .z = 0 },
            &.{ .w = 1, .y = 0, .x = 0, .z = 0 },
            c.JPH_MotionType_Static,
            OBJECT_LAYER_NON_MOVING,
        );
        defer c.JPH_BodyCreationSettings_Destroy(settings);
        const body_id = c.JPH_BodyInterface_CreateAndAddBody(body_interface, settings, c.JPH_Activation_DontActivate);
        break :blk body_id;
    };
    defer c.JPH_BodyInterface_RemoveAndDestroyBody(body_interface, floor_id);

    const sphere_id = blk: {
        const shape = c.JPH_SphereShape_Create(0.5).?;

        const settings = c.JPH_BodyCreationSettings_Create3(
            @ptrCast(shape),
            &.{ .x = 0, .y = 2, .z = 0 },
            &.{ .w = 1, .x = 0, .y = 0, .z = 0 },
            c.JPH_MotionType_Dynamic,
            OBJECT_LAYER_MOVING,
        );
        defer c.JPH_BodyCreationSettings_Destroy(settings);
        const body_id = c.JPH_BodyInterface_CreateAndAddBody(body_interface, settings, c.JPH_Activation_Activate);
        c.JPH_BodyInterface_SetRestitution(body_interface, body_id, 0.8);
        break :blk body_id;
    };
    defer c.JPH_BodyInterface_RemoveAndDestroyBody(body_interface, sphere_id);

    while (!c.WindowShouldClose()) {
        const err = c.JPH_PhysicsSystem_Update(physics_system, DELTA_TIME, 1, job_system);
        if (err != c.JPH_PhysicsUpdateError_None) {
            @panic("Failed to update JPH");
        }

        if (c.IsMouseButtonDown(c.MOUSE_BUTTON_RIGHT)) {
            c.UpdateCamera(&camera, c.CAMERA_FREE);
        }

        c.BeginDrawing();
        defer c.EndDrawing();

        c.ClearBackground(c.BLACK);
        c.BeginMode3D(camera);
        {
            var position: c.JPH_Vec3 = undefined;
            c.JPH_BodyInterface_GetCenterOfMassPosition(body_interface, sphere_id, &position);

            c.DrawSphere(.{ .x = position.x, .y = position.y, .z = position.z }, 0.5, c.RED);

            c.DrawGrid(10, 1);
        }
        c.EndMode3D();
        c.DrawFPS(0, 0);
    }
}
