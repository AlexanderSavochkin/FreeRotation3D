#include <SFML/Window.hpp>

//#pragma comment(lib, "glew.lib")
//#define GLEW_STATIC 1

#include <GL/glew.h>
#include <GL/gl.h>

#include <SFML/OpenGL.hpp>
#include <SFML/Graphics.hpp>

#include <fstream>
#include <string>

// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/ext.hpp>
using namespace glm;

#include "RotationalMechanics.h"
#include "QuaternionDirectIntegrationRotationalMechanics.h"
#include "MatrixRotationalMechanics.h"

using std::ifstream;
using std::string;


// Our vertices. Three consecutive floats give a 3D vertex; Three consecutive vertices give a triangle.
// A cube has 6 faces with 2 triangles each, so this makes 6*2=12 triangles, and 12*3 vertices
static const GLfloat g_vertex_buffer_data[] = {
    //Left
    -1.0f,-1.0f,-1.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.1109f, 0.5344f, // triangle 1 : begin
    -1.0f,-1.0f, 1.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.3931f, 0.5344f,
    -1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.3931f, 0.3040f, // triangle 1 : end

    //Left
    -1.0f,-1.0f,-1.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.1109f, 0.5344f,
    -1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.3931f, 0.3040f,
    -1.0f, 1.0f,-1.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.1109f, 0.3040f,

    //Back
    1.0f, 1.0f,-1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.7587f, 0.9867f, // triangle 2 : begin
    -1.0f,-1.0f,-1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.39852f, 0.7724f,
    -1.0f, 1.0f,-1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.39852f, 0.9867f, // triangle 2 : end

    //Back
    1.0f, 1.0f,-1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.7587f, 0.9867f,
    1.0f,-1.0f,-1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.7587f, 0.7724f,
    -1.0f,-1.0f,-1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.39852f, 0.7724f,

    //Bottom
    1.0f,-1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.7587f, 0.5249f,
    -1.0f,-1.0f,-1.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.39852f, 0.7620f,
    1.0f,-1.0f,-1.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.7587f, 0.7620f,

    //Bottom
    1.0f,-1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.7587f, 0.5249f,
    -1.0f,-1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.39852f, 0.5249f,
    -1.0f,-1.0f,-1.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.39852f, 0.7620f,

    //Front
    -1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.39852f, 0.3040f,
    -1.0f,-1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.39852f, 0.5344f,
    1.0f,-1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.7587f, 0.5344f,

    //Front
    1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.7587f, 0.3040f,
    -1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.39852f, 0.3040f,
    1.0f,-1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.7587f, 0.5344f,

    //Right
    1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.7655f, 0.3040f,
    1.0f,-1.0f,-1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.9745f, 0.5344f,
    1.0f, 1.0f,-1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.9745f, 0.3040f,

    //Right
    1.0f,-1.0f,-1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.9745f, 0.5344f,
    1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.7655f, 0.3040f,
    1.0f,-1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.7655f, 0.5344f,

    //Top
    1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.7587f, 0.2798f,
    1.0f, 1.0f,-1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.7587f, 0.0166f,
    -1.0f, 1.0f,-1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.39852f, 0.0166f,

    //Top
    1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.7587f, 0.2798f,
    -1.0f, 1.0f,-1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.39852f, 0.0166f,
    -1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.39852f, 0.2798f,
};

const char texturename[] = "texture.jpg";

glm::vec3 camera_position(4, 3, -3);

GLuint LoadShaders(const char * vertex_file_path, const char * fragment_file_path) {

    // Create the shaders
    GLuint VertexShaderID = glCreateShader(GL_VERTEX_SHADER);
    GLuint FragmentShaderID = glCreateShader(GL_FRAGMENT_SHADER);

    // Read the Vertex Shader code from the file
    std::string VertexShaderCode;
    std::ifstream VertexShaderStream(vertex_file_path, std::ios::in);
    if (VertexShaderStream.is_open()) {
        std::string Line = "";
        while (getline(VertexShaderStream, Line))
            VertexShaderCode += "\n" + Line;
        VertexShaderStream.close();
    }
    else {
        printf("Impossible to open %s. Are you in the right directory ? Don't forget to read the FAQ !\n", vertex_file_path);
        getchar();
        return 0;
    }

    // Read the Fragment Shader code from the file
    std::string FragmentShaderCode;
    std::ifstream FragmentShaderStream(fragment_file_path, std::ios::in);
    if (FragmentShaderStream.is_open()) {
        std::string Line = "";
        while (getline(FragmentShaderStream, Line))
            FragmentShaderCode += "\n" + Line;
        FragmentShaderStream.close();
    }

    GLint Result = GL_FALSE;
    int InfoLogLength;

    // Compile Vertex Shader
    printf("Compiling shader : %s\n", vertex_file_path);
    char const * VertexSourcePointer = VertexShaderCode.c_str();
    glShaderSource(VertexShaderID, 1, &VertexSourcePointer, NULL);
    glCompileShader(VertexShaderID);

    // Check Vertex Shader
    glGetShaderiv(VertexShaderID, GL_COMPILE_STATUS, &Result);
    glGetShaderiv(VertexShaderID, GL_INFO_LOG_LENGTH, &InfoLogLength);
    if (InfoLogLength > 0) {
        std::vector<char> VertexShaderErrorMessage(InfoLogLength + 1);
        glGetShaderInfoLog(VertexShaderID, InfoLogLength, NULL, &VertexShaderErrorMessage[0]);
        printf("%s\n", &VertexShaderErrorMessage[0]);
    }

    // Compile Fragment Shader
    printf("Compiling shader : %s\n", fragment_file_path);
    char const * FragmentSourcePointer = FragmentShaderCode.c_str();
    glShaderSource(FragmentShaderID, 1, &FragmentSourcePointer, NULL);
    glCompileShader(FragmentShaderID);

    // Check Fragment Shader
    glGetShaderiv(FragmentShaderID, GL_COMPILE_STATUS, &Result);
    glGetShaderiv(FragmentShaderID, GL_INFO_LOG_LENGTH, &InfoLogLength);
    if (InfoLogLength > 0) {
        std::vector<char> FragmentShaderErrorMessage(InfoLogLength + 1);
        glGetShaderInfoLog(FragmentShaderID, InfoLogLength, NULL, &FragmentShaderErrorMessage[0]);
        printf("%s\n", &FragmentShaderErrorMessage[0]);
    }

    // Link the program
    printf("Linking program\n");
    GLuint ProgramID = glCreateProgram();
    glAttachShader(ProgramID, VertexShaderID);
    glAttachShader(ProgramID, FragmentShaderID);
    glLinkProgram(ProgramID);

    // Check the program
    glGetProgramiv(ProgramID, GL_LINK_STATUS, &Result);
    glGetProgramiv(ProgramID, GL_INFO_LOG_LENGTH, &InfoLogLength);
    if (InfoLogLength > 0) {
        std::vector<char> ProgramErrorMessage(InfoLogLength + 1);
        glGetProgramInfoLog(ProgramID, InfoLogLength, NULL, &ProgramErrorMessage[0]);
        printf("%s\n", &ProgramErrorMessage[0]);
    }

    glDetachShader(ProgramID, VertexShaderID);
    glDetachShader(ProgramID, FragmentShaderID);

    glDeleteShader(VertexShaderID);
    glDeleteShader(FragmentShaderID);

    return ProgramID;
}

const char* const MATRIX_MODEL = "matrix";
const char* const QUATERNION_DI_MODEL = "quaternion_di";

RotationalMechanics* CreateRotationMechanicsModel(const string& model)
{
    if (model == QUATERNION_DI_MODEL)
    {
        return new QuaternionDirectIntegrationRotationalMechanics;
    }
    else
    {
        return new MatrixRotationalMechanics;
    }
}

int main(int argc, char **argv)
{
    float width = 3.0f, height = 2.0f, depth = 1.0f;

    string modelName = MATRIX_MODEL;
    if (argc >= 2)
    {
        modelName = argv[1];    
    }

    RotationalMechanics *mechanicalModel = CreateRotationMechanicsModel(modelName);
    mechanicalModel->setAngularMomentum(vec3(0.005f, 14.0f, 0.005f));
    mechanicalModel->setInertia(vec3(
        height*height + depth*depth, 
        width*width + depth*depth,
        height*height + width*width));

    //OpenGL Context
    sf::ContextSettings settings;
    settings.depthBits = 24;
    settings.stencilBits = 8;
    settings.antialiasingLevel = 4;
    settings.majorVersion = 3;
    settings.minorVersion = 3;

    // create the window
    sf::Window window(sf::VideoMode(800, 600), "OpenGL", sf::Style::Default, settings);
    window.setVerticalSyncEnabled(true);

    //Timer
    sf::Clock Clock;

    glewExperimental = true; // Needed in core profile
    if (glewInit() != GLEW_OK) {
        fprintf(stderr, "Failed to initialize GLEW\n");
        return -1;
    }

    // Enable depth test
    glEnable(GL_DEPTH_TEST);
    // Accept fragment if it closer to the camera than the former one
    glDepthFunc(GL_LESS);

    GLuint programID = LoadShaders("Shaders/simple.vertexshader", "Shaders/simple.fragmentshader ");

    // Get a handle for our "MVP" uniform
    GLuint MVPMatrixID = glGetUniformLocation(programID, "MVP");
    GLuint iMtMatrixID = glGetUniformLocation(programID, "iMt");

    // Projection matrix : 45° Field of View, 4:3 ratio, display range : 0.1 unit <-> 100 units
    glm::mat4 Projection = glm::perspective(45.0f, 4.0f / 3.0f, 0.1f, 100.0f);
    // Camera matrix
    glm::mat4 View = glm::lookAt(
        camera_position, // Camera is at (4,3,-3), in World Space
        glm::vec3(0, 0, 0), // and looks at the origin
        glm::vec3(0, 1, 0)  // Head is up (set to 0,-1,0 to look upside-down)
        );

    // This will identify our vertex buffer
    GLuint vertexbuffer;
    // Generate 1 buffer, put the resulting identifier in vertexbuffer
    glGenBuffers(1, &vertexbuffer);
    // The following commands will talk about our 'vertexbuffer' buffer
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);

    GLuint vertexarray;
    glGenVertexArrays(1, &vertexarray);
    glBindVertexArray(vertexarray);

    // Give our vertices to OpenGL.
    glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data), g_vertex_buffer_data, GL_STATIC_DRAW);

    glVertexAttribPointer(
        0,                  // attribute 0. Must match the layout in the shader.
        3,                  // size
        GL_FLOAT,           // type
        GL_FALSE,           // normalized?
        11 * sizeof(GLfloat),                  // stride
        0           // array buffer offset
    );
    glEnableVertexAttribArray(0);

    // 2nd attribute buffer : colors
    glVertexAttribPointer(
        1,                                // attribute. Must match the layout in the shader.
        3,                                // size
        GL_FLOAT,                         // type
        GL_FALSE,                         // normalized?
        11 * sizeof(GLfloat),                                // stride
        (GLvoid*)(3 * sizeof(GLfloat))                 // array buffer offset
    );
    glEnableVertexAttribArray(1);

    // 3rd attribute buffer : normals
    glVertexAttribPointer(
        2,                                // attribute. Must match the layout in the shader.
        3,                                // size
        GL_FLOAT,                         // type
        GL_FALSE,                         // normalized?
        11 * sizeof(GLfloat),                                // stride
        (GLvoid*)(6 * sizeof(GLfloat))                 // array buffer offset
    );
    glEnableVertexAttribArray(2);
    
    // 4th attribute buffer : texture coords
    glVertexAttribPointer(
        3,                                // attribute. Must match the layout in the shader.
        2,                                // size
        GL_FLOAT,                         // type
        GL_FALSE,                         // normalized?
        11 * sizeof(GLfloat),                                // stride
        (GLvoid*)(9 * sizeof(GLfloat))                 // array buffer offset
    );
    glEnableVertexAttribArray(3);
    
    glBindVertexArray(0);

    glUseProgram(programID);

    GLint light_color = glGetUniformLocation(programID, "light_color");
    GLint light_dir = glGetUniformLocation(programID, "light_dir");
    GLint camera_position_ref = glGetUniformLocation(programID, "view_position");

    glUniform3f(light_color, 1.0f, 1.0f, 1.0f);
    glUniform3f(light_dir, 0.0f, -1.0f, 1.0f);
    glUniform3f(camera_position_ref, camera_position.x, camera_position.y, camera_position.z);

    sf::Image img_data;
    if (!img_data.loadFromFile(texturename))
    {
        fprintf(stderr, "Could not load %s\n", texturename);
        return -1;
    }

    GLuint texture_handle;
    glGenTextures(1, &texture_handle);
    glBindTexture(GL_TEXTURE_2D, texture_handle);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);	// Set texture wrapping to GL_REPEAT
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    // Set texture filtering
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glTexImage2D(
        GL_TEXTURE_2D, 0, GL_RGBA,
        img_data.getSize().x, img_data.getSize().y,
        0,
        GL_RGBA, GL_UNSIGNED_BYTE, img_data.getPixelsPtr()
    );
    glGenerateMipmap(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, 0); // Unbind texture when done, so we won't accidentily mess up our texture.


    // run the main loop
    bool running = true;
    // Get elapsed time since last loop
    Clock.restart();
    while (running)
    {
        // Model matrix : an identity matrix (model will be at the origin)
        //glm::mat4 Model = glm::mat4(1.0f);
        // Our ModelViewProjection : multiplication of our 3 matrices
        const glm::mat4& Orientation = mechanicalModel->getOrientationMatrix();
        glm::mat4 MVP = Projection * View * Orientation * glm::scale(glm::mat4(1.0f), vec3(1.0, 1.73, 2.24)); // Remember, matrix multiplication is the other way around		
        glm::mat4 iMt = glm::inverseTranspose(Orientation);

        // handle events
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
            {
                // end the program
                running = false;
            }
            else if (event.type == sf::Event::Resized)
            {
                // adjust the viewport when the window is resized
            }
        }

        //Update bar orientation
        float dTIme = Clock.getElapsedTime().asSeconds();
        Clock.restart();
        mechanicalModel->update(dTIme);


        // clear the buffers
        glClearColor(0.2f, 0.2f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Use our shader
        //glUseProgram(programID);

        // Send our transformation to the currently bound shader, 
        // in the "MVP" uniform
        glUniformMatrix4fv(MVPMatrixID, 1, GL_FALSE, &MVP[0][0]);
        glUniformMatrix4fv(iMtMatrixID, 1, GL_FALSE, &iMt[0][0]);

        // draw...

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texture_handle);
        glUniform1i(glGetUniformLocation(programID, "texture1"), 0);


        // Draw the triangle !	
        glBindVertexArray(vertexarray);
        glDrawArrays(GL_TRIANGLES, 0, 12 * 3); // 12*3 indices starting at 0 -> 12 triangles -> 6 squares
        glBindVertexArray(0);

        // end the current frame (internally swaps the front and back buffers)
        window.display();
    }

    // release resources...
    delete mechanicalModel;

    return 0;
}