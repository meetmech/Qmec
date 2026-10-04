

cbuffer TransformBuffer : register(b0)
{
    row_major float4x4 model;
    row_major float4x4 modelViewProjection;
};


cbuffer CameraBuffer : register(b3)
{
    float3 cameraPosition;
    float cameraPadding;
};

cbuffer MaterialBuffer : register(b2)
{
    float4 albedoColor;
    float normalStrength;
    float metallic;
    float roughness;
    float specularLevel;
    uint hasAlbedoTexture;
    uint hasNormalTexture;
    float2 materialPadding;
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

Texture2D<float4> albedoTexture : register(t2);
Texture2D<float4> normalTexture : register(t3);
SamplerState materialSampler : register(s2);
Texture2D diffuseTexture : register(t0);
Texture2D<float> shadowTexture : register(t1);
SamplerState diffuseSampler : register(s0);
SamplerComparisonState shadowSampler : register(s1);

struct VertexInput
{
    float3 position : POSITION;
    float3 normal : NORMAL;
    float3 tangent : TANGENT;
    float3 color : COLOR;
    float2 uv : TEXCOORD;
};

struct VertexOutput
{
    float4 position : SV_POSITION;
    float4 worldPosition : TEXCOORD1;
    float4 lightPosition : TEXCOORD2;
    float3 worldNormal : NORMAL;
    float3 worldTangent : TANGENT;
    float3 vertexColor : COLOR;
    float2 uv : TEXCOORD;
};

VertexOutput VertexMain(VertexInput input)
{
    VertexOutput output;
    output.position = mul(float4(input.position, 1.0f), modelViewProjection);
    output.worldPosition = mul(float4(input.position, 1.0f), model);
    output.lightPosition = mul(output.worldPosition, lightViewProjection);
    output.worldNormal = mul(float4(input.normal, 0.0f), model).xyz;
    output.worldTangent = mul(float4(input.tangent,0.0f), model).xyz;
    output.vertexColor = input.color;
    output.uv = input.uv;
    return output;
}

static const float PI = 3.14159265359f;

float GetShadowVisibility(float4 lightPosition, float diffuseAmount)
{
    if (shadowsEnabled == 0U || lightPosition.w <= 0.0f)
    {
        return 1.0f;
    }

    float3 projected = lightPosition.xyz / lightPosition.w;
    float2 shadowUV = projected.xy * float2(0.5f, -0.5f) + 0.5f;
    if (any(shadowUV < 0.0f) || any(shadowUV > 1.0f) || projected.z < 0.0f || projected.z > 1.0f)
    {
        return 1.0f;
    }

    float bias = max(shadowDepthBias, shadowSlopeBias * (1.0f - diffuseAmount));
    return shadowTexture.SampleCmpLevelZero(shadowSampler, shadowUV, projected.z - bias);
}

float4 PixelMain(VertexOutput input) : SV_TARGET
{
    float4 textureColor = hasAlbedoTexture != 0U? albedoTexture.Sample(materialSampler, input.uv)
        : diffuseTexture.Sample(diffuseSampler, input.uv);
    float4 surfaceColor = float4(albedoColor.rgb * textureColor.rgb * input.vertexColor,albedoColor.a * textureColor.a);
    
    
    if (hasNormalTexture)
    {
        float3 normal = normalize(input.worldNormal);
        float3 tangent = normalize(input.worldTangent);
        float3 bitangent = cross(normal, tangent);

        float3 tangentNormal = normalTexture.Sample(materialSampler, input.uv).rgb * 2.0f - 1.0f;
        tangentNormal.xy *= normalStrength;

        input.worldNormal = normalize(tangentNormal.x * tangent + tangentNormal.y * bitangent + tangentNormal.z * normal);
    }
 

    float3 N = normalize(input.worldNormal);
    float3 L = normalize(lightDirection);
    float3 V = normalize(cameraPosition - input.worldPosition.xyz);
    float3 H = normalize(L + V);

    float NdotL = saturate(dot(N, L));
    float NdotV = saturate(dot(N, V));
    float NdotH = saturate(dot(N, H));
    float VdotH = saturate(dot(V, H));
    
    float safeRoughness = max(roughness, 0.04f);
    float alpha = safeRoughness * safeRoughness;
    float alpha2 = alpha * alpha;

    float denominator = NdotH * NdotH * (alpha2 - 1.0f) + 1.0f;

    float D = alpha2 / (PI * denominator * denominator);
    float3 dielectricF0 = float3(0.04f * specularLevel, 0.04f * specularLevel, 0.04f * specularLevel);
    float3 F0 = lerp(dielectricF0, surfaceColor.rgb, metallic);

    float3 F = F0 + (1.0f - F0) * pow(1.0f - VdotH, 5.0f);
    float k = (safeRoughness + 1.0f) * (safeRoughness + 1.0f) / 8.0f;

    float G1L = NdotL / (NdotL * (1.0f - k) + k);
    float G1V = NdotV / (NdotV * (1.0f - k) + k);

    float G = G1L * G1V;

    float3 specular = (D * F * G) / max(4.0f * NdotL * NdotV, 0.0001f);

    float3 diffuseWeight = (1.0f - F) * (1.0f - metallic);
    float3 diffuse = diffuseWeight * surfaceColor.rgb / PI;
    float visibility = GetShadowVisibility(input.lightPosition, NdotL);
    float3 directLighting = (diffuse + specular) * lightColor * NdotL * lightIntensity * visibility;
    float3 ambientLighting = diffuseWeight * surfaceColor.rgb * ambient;
    return float4(directLighting + ambientLighting, surfaceColor.a);
}
