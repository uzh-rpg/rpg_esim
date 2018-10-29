#version 330 core

#define M_PI 3.1415926535897932384626433832795

out vec4 FragColor;

// angular and linear camera velocity
uniform vec3 w_WC;
uniform vec3 v_WC;

// angular and linear object velocity
uniform bool dynamic_object;
uniform vec3 r_CB;
uniform vec3 w_WB;
uniform vec3 v_WB;

// camera intrinsics
uniform float fx;
uniform float fy;
uniform float cx;
uniform float cy;
uniform float width;
uniform float height;

// depth range used
uniform float near;
uniform float far;

// taken from: https://learnopengl.com/Advanced-OpenGL/Depth-testing
float LinearizeDepth(float depth)
{
    float z = depth * 2.0 - 1.0; // back to NDC
    return (2.0 * near * far) / (far + near - z * (far - near));
}

void main()
{
    float depth = LinearizeDepth(gl_FragCoord.z);
    float inv_z = 1.0f / depth;

    // calibrated coordinates
    float u = (gl_FragCoord.x - cx) / fx;
    float v = (gl_FragCoord.y - cy) / fy;

    // optic flow computation
    vec2 flow;
    if (dynamic_object)
    {
        float x = u*depth;
        float y = v*depth;
        float x_obj = x - r_CB.x;
        float y_obj = y - r_CB.y;
        float z_obj = depth - r_CB.z; // TODO, should not be done in loop, remains constant for object
        float vx = v_WB.x - v_WC.x - (w_WC.y * depth - w_WC.z * y) + (w_WB.y * z_obj - w_WB.z * y_obj);
        float vy = v_WB.y - v_WC.y - (w_WC.z * x - w_WC.x * depth) + (w_WB.z * x_obj - w_WB.x * z_obj);
        float vz = v_WB.z - v_WC.z - (w_WC.x * y - w_WC.y * x) + (w_WB.x * y_obj - w_WB.y * x_obj);

        flow = vec2(fx*(inv_z*vx - u*inv_z*vz), fy*(inv_z*vy - v*inv_z*vz));
    }
    else
    {
        // Eq. (3) in https://arxiv.org/pdf/1510.01972.pdf
        float vx_trans = -inv_z * v_WC.x
                        + u * inv_z * v_WC.z;

        float vx_rot = u * v * w_WC.x
                    - (1.0 + u * u) * w_WC.y
                    + v * w_WC.z;

        float vy_trans = -inv_z * v_WC.y
                        + v * inv_z * v_WC.z;

        float vy_rot = (1.0 + v * v) * w_WC.x
                    - u * v * w_WC.y
                    - u * w_WC.z;

        float vx = vx_trans + vx_rot;
        float vy = vy_trans + vy_rot;

        flow = vec2(fx * vx, fy * vy);
    }

    // red component: x component of the flow
    // green component: y component of the flow
    FragColor = vec4(flow.x, flow.y, 0.0f, 1.0f);
}
