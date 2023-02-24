#include "sph_demo.h"

void Sph_Demo::run_simulation(int number_of_frames, double fps)
{
    static constexpr unsigned int SCR_WIDTH = 720;
    static constexpr unsigned int SCR_HEIGHT = 720;
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // glfw window creation
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "PCISPH fluid simulation", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return;
    }
    glfwMakeContextCurrent(window);
    //glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    //glfwSetCursorPosCallback(window, mouse_callback);

    // tell GLFW to capture our mouse
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // glad: load all OpenGL function pointers
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return;
    }

    Shader shader("./render/shader/balls_vertex_shader.txt", "./render/shader/balls_fragment_shader.txt");
    Shader cylinder_shader("./render/shader/vertex_shader.txt", "./render/shader/fragment_shader.txt");

    Frame frame(0, 1.0 / fps);
    auto particles = _solver->get_sph_system_data();

    //process sphere mesh
    auto mesh = std::make_shared<Sphere_Mesh<>>(particles->get_particle_radius() * 0.95);
    mesh->initialize_vertices();

    unsigned int VBO_sphere, VAO_sphere, EBO_sphere;
    glGenVertexArrays(1, &VAO_sphere);
    glGenBuffers(1, &VBO_sphere);
    glGenBuffers(1, &EBO_sphere);
    
    // bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
    glBindVertexArray(VAO_sphere);
    glBindBuffer(GL_ARRAY_BUFFER, VBO_sphere);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * (mesh->X_SEGMENTS + 1) * (mesh->Y_SEGMENTS + 1) * 6, mesh->vertices, GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO_sphere);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * 6 * mesh->X_SEGMENTS * mesh->Y_SEGMENTS, mesh->indices, GL_STATIC_DRAW);
    
    // position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    // normal vector attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);


    //process cylinder mesh
    auto cylinder_mesh = std::make_shared<Cylinder_Mesh<>>(0.1, 0.75);
    cylinder_mesh->initialize_vertices();

    unsigned int VBO_cylinder, VAO_cylinder, EBO_cylinder;
    glGenVertexArrays(1, &VAO_cylinder);
    glGenBuffers(1, &VBO_cylinder);
    glGenBuffers(1, &EBO_cylinder);

    // bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
    glBindVertexArray(VAO_cylinder);
    glBindBuffer(GL_ARRAY_BUFFER, VBO_cylinder);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * (cylinder_mesh->X_SEGMENTS + 1) * (cylinder_mesh->Y_SEGMENTS + 1) * 6, cylinder_mesh->vertices, GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO_cylinder);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * 6 * cylinder_mesh->X_SEGMENTS * cylinder_mesh->Y_SEGMENTS, cylinder_mesh->indices, GL_STATIC_DRAW);

    // position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    // normal vector attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    glm::vec3 cylinder_positions[] = {
        glm::vec3(1., 0.375, 0.375),
        glm::vec3(1.5, 0.375, 0.75),
        glm::vec3(2., 0.375, 1.125)
    };

    unsigned int sphere_instance_VBO;
    glGenBuffers(1, &sphere_instance_VBO);

    // also set instance data to draw many particles
    glBindVertexArray(VAO_sphere);
    glBindBuffer(GL_ARRAY_BUFFER, sphere_instance_VBO); // this attribute comes from a different vertex buffer
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(2);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glVertexAttribDivisor(2, 1); // tell OpenGL this is an instanced vertex attribute.

    glEnable(GL_DEPTH_TEST);

    glm::mat4 model = glm::mat4(1.0f);
    glm::mat4 view = glm::mat4(1.0f); // make sure to initialize matrix to identity matrix first
    glm::mat4 projection = glm::mat4(1.0f);
    projection = glm::perspective(glm::radians(45.0f), static_cast<float>(SCR_WIDTH) / static_cast<float>(SCR_HEIGHT), 0.1f, 100.0f);
    //view = camera.get_view_matrix();
    view = glm::lookAt(glm::vec3(8.f, 5.0f, 5.0f),
        glm::vec3(0.0f, 0.0f, 0.0f),
        glm::vec3(0.0f, 1.0f, 0.0f));

    while(!glfwWindowShouldClose(window) && frame.index < number_of_frames) //render and simuation loop
    {
        processInput(window);

        _solver->update(frame);
        auto number_of_particles = particles->get_number_of_particles();
        std::cout << "frame: " << frame.index << ", number of particles: " << number_of_particles << std::endl;

        // render
        // ------
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // render particles
        shader.use();
        model = glm::mat4(1.0f);
        shader.set_matrix4f("model", model);
        shader.set_matrix4f("view", view);
        shader.set_matrix4f("projection", projection);
        shader.set_float3("objectColor", 0.2f, 0.2f, 0.75f);
        shader.set_float3("lightColor", 1.0f, 1.0f, 1.0f);
        shader.set_float3("lightPos", 0.0f, 1.0f, 2.0f);
        shader.set_float3("viewPos", 8.0f, 5.0f, 5.0f);

        glBindVertexArray(VAO_sphere);
        glBindBuffer(GL_ARRAY_BUFFER, sphere_instance_VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * number_of_particles, &(particles->get_positions_float()[0]), GL_STREAM_DRAW);
        glDrawElementsInstanced(GL_TRIANGLES, 6 * mesh->X_SEGMENTS * mesh->Y_SEGMENTS, GL_UNSIGNED_INT, 0, number_of_particles);
        glBindVertexArray(0);


        // render cylinders
        cylinder_shader.use();
        glBindVertexArray(VAO_cylinder);

        cylinder_shader.set_matrix4f("view", view);
        cylinder_shader.set_matrix4f("projection", projection);
        cylinder_shader.set_float3("objectColor", 0.7f, 0.2f, 0.2f);
        cylinder_shader.set_float3("lightColor", 1.0f, 1.0f, 1.0f);
        cylinder_shader.set_float3("lightPos", 0.0f, 1.0f, 2.0f);
        cylinder_shader.set_float3("viewPos", 8.0f, 5.0f, 5.0f);
        for (size_t i = 0; i < 3; ++i)
        {
            glm::mat4 cylinder_model = glm::translate(model, cylinder_positions[i]);
            cylinder_shader.set_matrix4f("model", cylinder_model);

            glDrawElements(GL_TRIANGLES, 6 * cylinder_mesh->X_SEGMENTS * cylinder_mesh->Y_SEGMENTS, GL_UNSIGNED_INT, 0);
        }
        glBindVertexArray(0);

        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        // -------------------------------------------------------------------------------
        glfwSwapBuffers(window);
        glfwPollEvents();

        ++frame;
    }
    glfwTerminate();
}

// build solver in this constructor
Sph_Demo::Sph_Demo(double target_spacing, int number_of_frames, double fps) : camera(glm::vec3(8.f, 5.f, 5.f))
{
    _target_spacing = target_spacing;
    _number_of_frames = number_of_frames;
    _fps = fps;

    BoundingBox3D domain(Vector3D::Zero(), Vector3D(3, 2, 1.5));
    double lz = domain.depth();

    double water_density = 1000.; //  kg/m^3
    _solver = std::make_shared<Pcisph_Solver3>(water_density, target_spacing);
    auto particles = _solver->get_sph_system_data();
    //particles->set_target_density(1000.);
    //particles->set_target_spacing(target_spacing);
    _solver->set_pseudo_viscosity_coefficient(0.);
    _solver->set_timestep_limit_scale(10.);
    particles->set_particle_mass(water_density * 4/3 * PI_d * target_spacing * target_spacing * target_spacing * 1.8 * 1.8 * 1.8);

    //build emitter
    BoundingBox3D source_bound(domain);
    source_bound.expand(-target_spacing);

    auto box1 = std::make_shared<Box3>(Vector3D::Zero(), Vector3D(0.5 + 0.001, 0.75 + 0.001, 0.75 * lz + 0.001));
    auto box2 = std::make_shared<Box3>(Vector3D(2.5 - 0.001, 0., 0.25 * lz - 0.001), Vector3D(3.5 + 0.001, 0.75 + 0.001, 1.5 * lz + 0.001));

    auto box_set = std::make_shared<ImplicitSurfaceSet3>();
    box_set->set_explicit_surfaces({ box1, box2 });

    auto emitter = std::make_shared<VolumeParticleEmitter3>(box_set, source_bound, target_spacing);

    _solver->set_emitter(emitter);

    //build collider
    auto cyl1 = std::make_shared<Cylinder3>(Vector3D(1., 0.375, 0.375), 0.1, 0.75);
    auto cyl2 = std::make_shared<Cylinder3>(Vector3D(1.5, 0.375, 0.75), 0.1, 0.75);
    auto cyl3 = std::make_shared<Cylinder3>(Vector3D(2., 0.375, 1.125), 0.1, 0.75);

    auto box = std::make_shared<Box3>(domain, Transform3(), true);

    auto surface_set = std::make_shared<ImplicitSurfaceSet3>(std::vector<Surface3Ptr>({ cyl1,cyl2,cyl3,box }));
    //surface_set->set_explicit_surfaces({ cyl1,cyl2,cyl3,box });

    auto collider = std::make_shared<Rigid_Body_Collider3>(surface_set);

    _solver->set_collider(collider);
}

Sph_Demo::~Sph_Demo()
{
}

void Sph_Demo::run()
{
    run_simulation(_number_of_frames, _fps);
}



// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
void processInput(GLFWwindow* window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
}