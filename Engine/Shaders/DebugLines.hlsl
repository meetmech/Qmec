cbuffer DebugCamera : register(b0)
{
    row_major float4x4 viewProjection;
};

struct VertexInput
{
    float3 position : POSITION;
    float3 color : COLOR;
};

struct VertexOutput
{
    float4 position : SV_POSITION;
    float3 color : COLOR;
};

VertexOutput VertexMain(VertexInput input)
{
    VertexOutput output;
    output.position = mul(float4(input.position, 1.0f), viewProjection);
    output.color = input.color;
    return output;
}

float4 PixelMain(VertexOutput input) : SV_TARGET
{
    return float4(input.color, 1.0f);
}
