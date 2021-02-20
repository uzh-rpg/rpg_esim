#version 330 core
out vec4 FragColor;

in vec2 TexCoords;
in vec3 Normal;

uniform sampler2D texture_diffuse1;

uniform float alpha_ambient;

void main()
{
    // global light (sun)
    vec3 LightDirection = vec3(0.8, -0.8, 1);

    // material properties
    vec4 MaterialColor = texture(texture_diffuse1, TexCoords);


    // diffuse part
    float diffuse_frac = dot(normalize(Normal), normalize(LightDirection));

    float diffuse =
          clamp(diffuse_frac, 0, 1) // front
        + clamp(diffuse_frac * 0.3, -1, 0); // back

    FragColor =
          MaterialColor * alpha_ambient // ambient
        + MaterialColor * (1-alpha_ambient) * diffuse;
}
