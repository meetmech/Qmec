Texture2D sceneTexture : register(t0);
SamplerState sceneSampler : register(s0);

struct VertexOutput
{
    float4 position : SV_POSITION;
    float2 uv : TEXCOORD0;
};

cbuffer PostProcessBuffer : register(b2)
{
    float brightness;
    float contrast;
    float saturation;
    float hueShift;
};

VertexOutput VertexMain(uint vertexId : SV_VertexID)
{
    const float2 positions[3] =
    {
        float2(-1.0f, -1.0f),
        float2(-1.0f, 3.0f),
        float2(3.0f, -1.0f)
    };

    VertexOutput output;

    const float2 position = positions[vertexId];

    output.position = float4(position, 0.0f, 1.0f);

    output.uv = float2(position.x * 0.5f + 0.5f,0.5f - position.y * 0.5f);

    return output;
}


float3 RGBToHSV(float3 color)
{
    const float4 K = float4(0.0f,-1.0f / 3.0f,2.0f / 3.0f,-1.0f);

    float4 P = lerp(float4(color.bg, K.wz),float4(color.gb, K.xy),step(color.b, color.g));

    float4 Q = lerp(float4(P.xyw, color.r),float4(color.r, P.yzx),step(P.x, color.r));

    const float epsilon = 1e-10f;

    float difference = Q.x - min(Q.w, Q.y);

    return float3(abs(Q.z +(Q.w - Q.y) /(6.0f * difference + epsilon)),difference / (Q.x + epsilon),Q.x);
}


float3 HSVToRGB(float3 hsv)
{
    const float3 K = float3(1.0f, 2.0f / 3.0f, 1.0f / 3.0f);
    float3 P = abs(frac(hsv.xxx + K) * 6.0f - 3.0f);

    return hsv.z * lerp(K.xxx,saturate(P - 1.0f),hsv.y);
}


float3 ApplyHue(float3 color, float hueShift)
{
    float3 hsv = RGBToHSV(color);

    hsv.x = frac(hsv.x + hueShift);

    return HSVToRGB(hsv);
}


float4 PixelMain(VertexOutput input) : SV_TARGET
{
    float4 BaseColor = sceneTexture.Sample(sceneSampler,input.uv);

    // Brightness
    BaseColor *= brightness;

    // Contrast
    BaseColor = (BaseColor - 0.5f) * (1.0f + contrast) + 0.5f;
    
    float luminance = dot(BaseColor.xyz,float3(0.2126f,0.7152f,0.0722f));

    BaseColor.xyz =luminance +(BaseColor.xyz - luminance) * saturation;

    // Hue
    BaseColor.xyz = ApplyHue(BaseColor.xyz, hueShift);

    // Keep final color inside displayable range
    BaseColor = saturate(BaseColor);

    return BaseColor;
}