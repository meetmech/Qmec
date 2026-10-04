  Texture2D diffuseTexture : register(t0);
SamplerState diffuseSampler : register(s0);
Texture2D<float> shadowTexture : register(t1);
SamplerComparisonState shadowSampler : register(s1);
  
 cbuffer TransformBuffer : register(b0)
    {
       row_major float4x4 model;
       row_major float4x4 modelViewProjection;
    };

  cbuffer LightBuffer : register(b1)
{
    float3 lightDirection;
    float ambient;
  
    float3 lightColor;
    float lightIntensity;
    
    
    row_major float4x4 lightViewProjection;
    float shadowDepthBias;
    float shadowSlopeBias;
    uint shadowsEnabled;
    float lightPadding;
}


struct VertexInput
{
    float3 position : POSITION;
    float3 normal : NORMAL;
    float3 color : COLOR;
     float2 uv : TEXCOORD;
};

struct VertexOutput
{
    float4 position : SV_POSITION;
    float4 lightPosition : TEXCOORD1;
    float3 normal : NORMAL;
    float3 color : COLOR;
     float2 uv : TEXCOORD;
};

VertexOutput VertexMain(VertexInput input)
{
    VertexOutput output;
    output.position = mul(float4(input.position, 1.0F),modelViewProjection);
    float4 worldPosition = mul(float4(input.position, 1.0f), model);
    output.lightPosition = mul(worldPosition, lightViewProjection);
    output.normal = mul(float4(input.normal,0.0f),model).xyz;
    output.uv = input.uv;
    output.color = input.color;
    return output;
}

float GetShadowVisibility(float4 lightPosition, float diffuse)
{
    if (shadowsEnabled == 0U || lightPosition.w <= 0.0f)
    {
        return 1.0f;
    }

    float3 projected = lightPosition.xyz / lightPosition.w;
    float2 shadowUV = projected.xy * float2(0.5f, -0.5f) + 0.5f;
    if (any(shadowUV < 0.0f) || any(shadowUV > 1.0f) ||
        projected.z < 0.0f || projected.z > 1.0f)
    {
        return 1.0f;
    }

    float bias = max(shadowDepthBias, shadowSlopeBias * (1.0f - diffuse));
    return shadowTexture.SampleCmpLevelZero(shadowSampler, shadowUV, projected.z - bias);
}

float4 PixelMain(VertexOutput input) : SV_TARGET
{
    float3 normal = normalize(input.normal);

    float diffuse = saturate(dot(normal, lightDirection));
    float visibility = GetShadowVisibility(input.lightPosition, diffuse);
    float3 brightness = ambient +(1.0f - ambient) * diffuse * lightColor * lightIntensity * visibility;
    float4 textureColor =diffuseTexture.Sample(diffuseSampler, input.uv);

    return float4(input.color * textureColor.rgb * brightness,textureColor.a);
}
