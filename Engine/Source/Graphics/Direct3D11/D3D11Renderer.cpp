#include "QMEC/Graphics/D3D11Renderer.h"
#include "QMEC/Assets/ImageData.h"
#include "QMEC/Math/Mat4.h"
#include "QMEC/Assets/AssetSystem.h"
#include "QMEC/Scene/Scene.h"
#include "QMEC/Scene/Camera.h"
#include "QMEC/Scene/Components/CameraComponent.h"
#include "QMEC/Scene/Components/TransformComponent.h"
#include "QMEC/Scene/Components/MeshRendererComponent.h"
#include "QMEC/Scene/Components/DirectionalLightComponent.h"
#include "QMEC/Scene/Components/MaterialComponent.h"
#include <Windows.h>
#include <d3d11.h>
#include <d3dcompiler.h>
#include <dxgi.h>
#include <wrl/client.h>
#include <cmath>
#include <memory>
#include <numbers>
#include <algorithm>
#include <cstddef>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>


namespace
{
    struct DebugVertex
    {
        qmec::Vec3 position;
        qmec::Vec3 color;
    };
    static_assert(sizeof(DebugVertex) == 24U);

    struct alignas(16) TransformConstants
    {
        qmec::Mat4 model = qmec::Mat4::Identity();
        qmec::Mat4 modelViewProjection = qmec::Mat4::Identity();
    };

    static_assert(sizeof(TransformConstants) == 128U);

    struct alignas(16) LightConstants
    {
        float direction[3]{ 0.0f, 0.0f, -1.0f };
        float ambient{ 0.15f };

        float color[3]{ 1.0f, 1.0f, 1.0f };
        float intensity{ 0.0f }; 
        qmec::Mat4 lightViewProjection = qmec::Mat4::Identity();
        float shadowDepthBias{0.0005f};
        float shadowSlopeBias{0.002f};
        std::uint32_t shadowsEnabled{0U};
        float padding{};
    };

    static_assert(sizeof(LightConstants) == 112U);

    struct alignas(16) MaterialConstants
    {
        float albedoColor[4]{1.0f, 1.0f, 1.0f, 1.0f};
        float normalStrength{1.0f};
        float metallic{};
        float roughness{0.5f};
        float specularLevel{0.5f};
        std::uint32_t hasAlbedoTexture{};
        std::uint32_t hasNormalTexture{};
        float padding[2]{};
    };

    static_assert(sizeof(MaterialConstants) == 48U);

    struct alignas(16) CameraConstants
    {
        float position[3]{};
        float padding{};
    };

    static_assert(sizeof(CameraConstants) == 16U);

    

    

}

namespace qmec
{



    using Microsoft::WRL::ComPtr;

    struct D3D11Renderer::Implementation
    {
        struct MaterialTexture
        {
            ComPtr<ID3D11Texture2D> texture{};
            ComPtr<ID3D11ShaderResourceView> view{};
        };

        ComPtr<ID3D11Device> device{};
        ComPtr<ID3D11DeviceContext> deviceContext{};
        ComPtr<IDXGISwapChain> swapChain{};
        ComPtr<ID3D11RenderTargetView> renderTargetView{};
        ComPtr<ID3D11VertexShader> vertexShader{};
        ComPtr<ID3D11PixelShader> pixelShader{};
        ComPtr<ID3D11InputLayout> inputLayout{};
        ComPtr<ID3D11VertexShader> debugVertexShader{};
        ComPtr<ID3D11PixelShader> debugPixelShader{};
        ComPtr<ID3D11InputLayout> debugInputLayout{};
        ComPtr<ID3D11Buffer> debugVertexBuffer{};
        ComPtr<ID3D11Buffer> debugViewProjectionBuffer{};
        ComPtr<ID3D11DepthStencilState> debugDepthState{};
        UINT debugVertexCapacity{0U};
        ComPtr<ID3D11Buffer> transformConstantBuffer{};
        ComPtr<ID3D11Buffer> lightConstantBuffer{};
        ComPtr<ID3D11Buffer> materialConstantBuffer{};
        ComPtr<ID3D11Buffer> cameraConstantBuffer{};
        ComPtr<ID3D11Texture2D> diffuseTexture{};
        ComPtr<ID3D11ShaderResourceView> diffuseTextureView{};
        ComPtr<ID3D11SamplerState> diffuseSampler{};
        std::unordered_map<std::string, MaterialTexture> albedoTextureCache{};
        std::unordered_map<std::string, MaterialTexture> normalTextureCache{};
        ComPtr<ID3D11Texture2D> depthStencilTexture{};
        ComPtr<ID3D11DepthStencilView> depthStencilView{};
        ComPtr<ID3D11Texture2D> lightDepthTexture{};
        ComPtr<ID3D11DepthStencilView> lightDepthStencilView{};
        ComPtr<ID3D11ShaderResourceView> lightTextureView{};
        ComPtr<ID3D11SamplerState> shadowSampler{};
        std::uint32_t shadowMapResolution{1024U};
        D3D11_VIEWPORT cameraViewport{};



        float aspectRatio{1.0f};
    };

    D3D11Renderer::D3D11Renderer(): implementation_(std::make_unique<Implementation>())
    {
    }

    D3D11Renderer::~D3D11Renderer() noexcept = default;

    ComPtr<ID3D11Device> D3D11Renderer::GetDevice() const noexcept
    {
        return implementation_->device;
    }

    bool D3D11Renderer::Initialize(void* nativeWindowHandle,std::uint32_t width,std::uint32_t height) noexcept
    {
        return Initialize(nativeWindowHandle, width, height, nullptr);
    }

    bool D3D11Renderer::Initialize(
        void* nativeWindowHandle,
        std::uint32_t width,
        std::uint32_t height,
        ID3D11Device* sharedDevice) noexcept
    {
        if(nativeWindowHandle == nullptr || width == 0U || height == 0U)
        {
            return false;
        }

        const float aspectRatio = static_cast<float>(width) / static_cast<float>(height);

        implementation_->aspectRatio = aspectRatio;

        DXGI_SWAP_CHAIN_DESC swapChainDescription{};
        swapChainDescription.BufferDesc.Width = width;
        swapChainDescription.BufferDesc.Height = height;
        swapChainDescription.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
        swapChainDescription.SampleDesc.Count = 1U;
        swapChainDescription.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
        swapChainDescription.BufferCount = 2U;
        swapChainDescription.OutputWindow = static_cast<HWND>(nativeWindowHandle);
        swapChainDescription.Windowed = TRUE;
        swapChainDescription.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;

        D3D_FEATURE_LEVEL selectedFeatureLevel{};
        HRESULT deviceResult = E_FAIL;
        if (sharedDevice == nullptr)
        {
            deviceResult = D3D11CreateDeviceAndSwapChain(
                nullptr,
                D3D_DRIVER_TYPE_HARDWARE,
                nullptr,
                0U,
                nullptr,
                0U,
                D3D11_SDK_VERSION,
                &swapChainDescription,
                implementation_->swapChain.GetAddressOf(),
                implementation_->device.GetAddressOf(),
                &selectedFeatureLevel,
                implementation_->deviceContext.GetAddressOf());
        }
        else
        {
            implementation_->device = sharedDevice;
            implementation_->device->GetImmediateContext(
                implementation_->deviceContext.GetAddressOf());

            ComPtr<IDXGIDevice> dxgiDevice{};
            ComPtr<IDXGIAdapter> adapter{};
            ComPtr<IDXGIFactory> factory{};
            if (FAILED(implementation_->device.As(&dxgiDevice))
                || FAILED(dxgiDevice->GetAdapter(adapter.GetAddressOf()))
                || FAILED(adapter->GetParent(
                    IID_PPV_ARGS(factory.GetAddressOf()))))
            {
                return false;
            }

            deviceResult = factory->CreateSwapChain(
                implementation_->device.Get(),
                &swapChainDescription,
                implementation_->swapChain.GetAddressOf());
        }

        if(FAILED(deviceResult))
        {
            return false;
        }

        UINT shaderCompileFlags = D3DCOMPILE_ENABLE_STRICTNESS;

#if defined(_DEBUG)
        shaderCompileFlags |=D3DCOMPILE_DEBUG |D3DCOMPILE_SKIP_OPTIMIZATION;
#endif

        ComPtr<ID3DBlob> vertexShaderBytecode{};
        ComPtr<ID3DBlob> shaderErrors{};
        const HRESULT vertexCompileResult = D3DCompileFromFile(
            L"Shaders/MaterialTemplate.hlsl",
            nullptr,
            D3D_COMPILE_STANDARD_FILE_INCLUDE,
            "VertexMain",
            "vs_5_0",
            shaderCompileFlags,
            0U,
            vertexShaderBytecode.GetAddressOf(),
            shaderErrors.GetAddressOf());

        if(FAILED(vertexCompileResult))
        {
            if(shaderErrors != nullptr)
            {
                OutputDebugStringA(static_cast<const char*>(shaderErrors->GetBufferPointer()));
            }
            return false;
        }

        shaderErrors.Reset();
        ComPtr<ID3DBlob> pixelShaderBytecode{};
        const HRESULT pixelCompileResult = D3DCompileFromFile(
            L"Shaders/MaterialTemplate.hlsl",
            nullptr,
            D3D_COMPILE_STANDARD_FILE_INCLUDE,
            "PixelMain",
            "ps_5_0",
            shaderCompileFlags,
            0U,
            pixelShaderBytecode.GetAddressOf(),
            shaderErrors.GetAddressOf());

        if(FAILED(pixelCompileResult))
        {
            if(shaderErrors != nullptr)
            {
                OutputDebugStringA(static_cast<const char*>(shaderErrors->GetBufferPointer()));
            }
            return false;
        }

        const HRESULT vertexShaderResult = implementation_->device->CreateVertexShader(vertexShaderBytecode->GetBufferPointer(),
                vertexShaderBytecode->GetBufferSize(),
                nullptr,
                implementation_->vertexShader.GetAddressOf());
        if(FAILED(vertexShaderResult))
        {
            return false;
        }

        const HRESULT pixelShaderResult = implementation_->device->CreatePixelShader(
                pixelShaderBytecode->GetBufferPointer(),
                pixelShaderBytecode->GetBufferSize(),
                nullptr,
                implementation_->pixelShader.GetAddressOf());
        if(FAILED(pixelShaderResult))
        {
            return false;
        }

        implementation_->deviceContext->VSSetShader(implementation_->vertexShader.Get(),nullptr,0U);
        implementation_->deviceContext->PSSetShader(implementation_->pixelShader.Get(),nullptr, 0U);


       

        constexpr D3D11_INPUT_ELEMENT_DESC inputElements[] =
        {
            {
                "POSITION",
                0U,
                DXGI_FORMAT_R32G32B32_FLOAT,
                0U,
                0U,
                D3D11_INPUT_PER_VERTEX_DATA,
                0U
            },
             {
                "NORMAL",
                0U,
                DXGI_FORMAT_R32G32B32_FLOAT,
                0U,
                D3D11_APPEND_ALIGNED_ELEMENT,
                D3D11_INPUT_PER_VERTEX_DATA,
                0U
            },
             {
                "TANGENT",
                0U,
                DXGI_FORMAT_R32G32B32_FLOAT,
                0U,
                D3D11_APPEND_ALIGNED_ELEMENT,
                D3D11_INPUT_PER_VERTEX_DATA,
                0U
            },
            {
                "COLOR",
                0U,
                DXGI_FORMAT_R32G32B32_FLOAT,
                0U,
                D3D11_APPEND_ALIGNED_ELEMENT,
                D3D11_INPUT_PER_VERTEX_DATA,
                0U
            },
             {
                "TEXCOORD",
                0U,
                DXGI_FORMAT_R32G32_FLOAT,
                0U,
                D3D11_APPEND_ALIGNED_ELEMENT,
                D3D11_INPUT_PER_VERTEX_DATA,
                0U
            },
        };

        const HRESULT InputAssemblerBuild = implementation_->device->CreateInputLayout(inputElements,5U,
            vertexShaderBytecode->GetBufferPointer(), vertexShaderBytecode->GetBufferSize(),
            implementation_->inputLayout.GetAddressOf());

        if (FAILED(InputAssemblerBuild))
        {
            return false;
        }

        implementation_->deviceContext->IASetInputLayout(implementation_->inputLayout.Get());

       
        implementation_->deviceContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);


        D3D11_BUFFER_DESC constantBufferDescription{}; 
        constantBufferDescription.ByteWidth = static_cast<UINT>(sizeof(TransformConstants));
        constantBufferDescription.Usage = D3D11_USAGE_DYNAMIC;
        constantBufferDescription.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
        constantBufferDescription.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;

       

        const HRESULT constantBufferResult =implementation_->device->CreateBuffer(&constantBufferDescription,nullptr,
                implementation_->transformConstantBuffer.GetAddressOf());

        if (FAILED(constantBufferResult))
        {
            return false;
        }

        ID3D11Buffer* constantBuffers[] =
        {
            implementation_->transformConstantBuffer.Get()
        };

        implementation_->deviceContext->VSSetConstantBuffers(0U,1U,constantBuffers);


        const LightConstants initialLight{};


        D3D11_BUFFER_DESC lightBufferDescription{};
        lightBufferDescription.ByteWidth = static_cast<UINT>(sizeof(LightConstants));
        lightBufferDescription.Usage = D3D11_USAGE_DYNAMIC;
        lightBufferDescription.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
        lightBufferDescription.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;

        D3D11_SUBRESOURCE_DATA lightData{};
        lightData.pSysMem = &initialLight;

        const HRESULT lightBufferResult = implementation_->device->CreateBuffer(
                &lightBufferDescription,
                &lightData,
                implementation_->lightConstantBuffer.GetAddressOf());

        if (FAILED(lightBufferResult))
        {
            return false;
        }

        ID3D11Buffer* lightBuffers[] =
        {
            implementation_->lightConstantBuffer.Get()
        };

        implementation_->deviceContext->PSSetConstantBuffers(
            1U,
            1U,
            lightBuffers);
        implementation_->deviceContext->VSSetConstantBuffers(1U, 1U, lightBuffers);

        const MaterialConstants initialMaterial{};
        D3D11_BUFFER_DESC materialBufferDescription{};
        materialBufferDescription.ByteWidth = static_cast<UINT>(sizeof(MaterialConstants));
        materialBufferDescription.Usage = D3D11_USAGE_DYNAMIC;
        materialBufferDescription.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
        materialBufferDescription.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
        D3D11_SUBRESOURCE_DATA materialData{};
        materialData.pSysMem = &initialMaterial;
        if (FAILED(implementation_->device->CreateBuffer(
            &materialBufferDescription, &materialData,
            implementation_->materialConstantBuffer.GetAddressOf())))
        {
            return false;
        }
        ID3D11Buffer* materialBuffer = implementation_->materialConstantBuffer.Get();
        implementation_->deviceContext->PSSetConstantBuffers(2U, 1U, &materialBuffer);

        const CameraConstants initialCamera{};
        D3D11_BUFFER_DESC cameraBufferDescription{};
        cameraBufferDescription.ByteWidth = static_cast<UINT>(sizeof(CameraConstants));
        cameraBufferDescription.Usage = D3D11_USAGE_DYNAMIC;
        cameraBufferDescription.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
        cameraBufferDescription.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
        D3D11_SUBRESOURCE_DATA cameraData{};
        cameraData.pSysMem = &initialCamera;
        if (FAILED(implementation_->device->CreateBuffer(
            &cameraBufferDescription, &cameraData,
            implementation_->cameraConstantBuffer.GetAddressOf())))
        {
            return false;
        }
        ID3D11Buffer* cameraBuffer = implementation_->cameraConstantBuffer.Get();
        implementation_->deviceContext->PSSetConstantBuffers(3U, 1U, &cameraBuffer);

        ImageData diffuseImage{};
        if(!LoadImageRgba8( "Assets/Textures/MicrosoftTeams-image (1).png",diffuseImage))
        {
            OutputDebugStringA("QMEC failed to load the diffuse texture.\n");
            return false;
        }

        D3D11_TEXTURE2D_DESC textureDescription{};
        textureDescription.Width = diffuseImage.width;
        textureDescription.Height = diffuseImage.height;
        textureDescription.MipLevels = 1U;
        textureDescription.ArraySize = 1U;
        textureDescription.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
        textureDescription.SampleDesc.Count = 1U;
        textureDescription.Usage = D3D11_USAGE_IMMUTABLE;
        textureDescription.BindFlags = D3D11_BIND_SHADER_RESOURCE;

        D3D11_SUBRESOURCE_DATA textureData{};
        textureData.pSysMem = diffuseImage.pixels.data();
        textureData.SysMemPitch = diffuseImage.width * 4U;

        const HRESULT textureResult = implementation_->device->CreateTexture2D(&textureDescription,&textureData,implementation_->diffuseTexture.GetAddressOf());

        if(FAILED(textureResult))
        {
            return false;
        }

        const HRESULT textureViewResult = implementation_->device->CreateShaderResourceView(
                implementation_->diffuseTexture.Get(),
                nullptr,
                implementation_->diffuseTextureView.GetAddressOf());

        if(FAILED(textureViewResult))
        {
            return false;
        }

        D3D11_SAMPLER_DESC samplerDescription{};
        samplerDescription.Filter = D3D11_FILTER_MIN_MAG_MIP_LINEAR;
        samplerDescription.AddressU = D3D11_TEXTURE_ADDRESS_WRAP;
        samplerDescription.AddressV = D3D11_TEXTURE_ADDRESS_WRAP;
        samplerDescription.AddressW = D3D11_TEXTURE_ADDRESS_WRAP;
        samplerDescription.MaxLOD = D3D11_FLOAT32_MAX;

        const HRESULT samplerResult =implementation_->device->CreateSamplerState(&samplerDescription,implementation_->diffuseSampler.GetAddressOf());

        if(FAILED(samplerResult))
        {
            return false;
        }

        D3D11_TEXTURE2D_DESC depthTextureDescription{};
        depthTextureDescription.Width = width;
        depthTextureDescription.Height = height;
        depthTextureDescription.MipLevels = 1U;
        depthTextureDescription.ArraySize = 1U;
        depthTextureDescription.Format = DXGI_FORMAT_D24_UNORM_S8_UINT;
        depthTextureDescription.SampleDesc.Count = 1U;
        depthTextureDescription.Usage = D3D11_USAGE_DEFAULT;
        depthTextureDescription.BindFlags = D3D11_BIND_DEPTH_STENCIL;

        const HRESULT depthTextureResult =implementation_->device->CreateTexture2D(&depthTextureDescription,nullptr,
                implementation_->depthStencilTexture.GetAddressOf());

        if (FAILED(depthTextureResult))
        {
            return false;
        }

        const HRESULT depthStencilViewResult =
            implementation_->device->CreateDepthStencilView( implementation_->depthStencilTexture.Get(),nullptr,implementation_->depthStencilView.GetAddressOf());

        if(FAILED(depthStencilViewResult))
        {
            return false;
        }

        D3D11_TEXTURE2D_DESC lightDepthDescription{};
        lightDepthDescription.Width = implementation_->shadowMapResolution;
        lightDepthDescription.Height = implementation_->shadowMapResolution;
        lightDepthDescription.MipLevels = 1U;
        lightDepthDescription.ArraySize = 1U;
        lightDepthDescription.Format = DXGI_FORMAT_R32_TYPELESS;
        lightDepthDescription.SampleDesc.Count = 1U;
        lightDepthDescription.Usage = D3D11_USAGE_DEFAULT;
        lightDepthDescription.BindFlags =  D3D11_BIND_DEPTH_STENCIL | D3D11_BIND_SHADER_RESOURCE;

        if (FAILED(implementation_->device->CreateTexture2D(
            &lightDepthDescription, nullptr,
            implementation_->lightDepthTexture.GetAddressOf())))
        {
            return false;
        }

        D3D11_DEPTH_STENCIL_VIEW_DESC lightDepthViewDescription{};
        lightDepthViewDescription.Format = DXGI_FORMAT_D32_FLOAT;
        lightDepthViewDescription.ViewDimension = D3D11_DSV_DIMENSION_TEXTURE2D;
        lightDepthViewDescription.Texture2D.MipSlice = 0U;

        if (FAILED(implementation_->device->CreateDepthStencilView(
            implementation_->lightDepthTexture.Get(), &lightDepthViewDescription,
            implementation_->lightDepthStencilView.GetAddressOf())))
        {
            return false;
        }

     
        D3D11_SHADER_RESOURCE_VIEW_DESC lightResourceViewDescription{};
        lightResourceViewDescription.Format = DXGI_FORMAT_R32_FLOAT;
        lightResourceViewDescription.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
        lightResourceViewDescription.Texture2D.MostDetailedMip = 0U;
        lightResourceViewDescription.Texture2D.MipLevels = 1U;

        if (FAILED(implementation_->device->CreateShaderResourceView(
            implementation_->lightDepthTexture.Get(), &lightResourceViewDescription,
            implementation_->lightTextureView.GetAddressOf())))
        {
            return false;
        }

        D3D11_SAMPLER_DESC shadowSamplerDescription{};
        shadowSamplerDescription.Filter = D3D11_FILTER_COMPARISON_MIN_MAG_LINEAR_MIP_POINT;
        shadowSamplerDescription.AddressU = D3D11_TEXTURE_ADDRESS_BORDER;
        shadowSamplerDescription.AddressV = D3D11_TEXTURE_ADDRESS_BORDER;
        shadowSamplerDescription.AddressW = D3D11_TEXTURE_ADDRESS_BORDER;
        shadowSamplerDescription.ComparisonFunc = D3D11_COMPARISON_LESS_EQUAL;
        shadowSamplerDescription.BorderColor[0] = 1.0f;
        shadowSamplerDescription.BorderColor[1] = 1.0f;
        shadowSamplerDescription.BorderColor[2] = 1.0f;
        shadowSamplerDescription.BorderColor[3] = 1.0f;
        shadowSamplerDescription.MaxAnisotropy = 1U;
        shadowSamplerDescription.MaxLOD = D3D11_FLOAT32_MAX;
        if (FAILED(implementation_->device->CreateSamplerState(
            &shadowSamplerDescription, implementation_->shadowSampler.GetAddressOf())))
        {
            return false;
        }

        ComPtr<ID3DBlob> debugVertexBytecode{};
        ComPtr<ID3DBlob> debugPixelBytecode{};
        shaderErrors.Reset();
        if (FAILED(D3DCompileFromFile(L"Shaders/DebugLines.hlsl", nullptr,
            D3D_COMPILE_STANDARD_FILE_INCLUDE, "VertexMain", "vs_5_0",
            shaderCompileFlags, 0U, debugVertexBytecode.GetAddressOf(),
            shaderErrors.GetAddressOf())))
        {
            if (shaderErrors)
                OutputDebugStringA(static_cast<const char*>(shaderErrors->GetBufferPointer()));
            return false;
        }
        shaderErrors.Reset();
        if (FAILED(D3DCompileFromFile(L"Shaders/DebugLines.hlsl", nullptr,
            D3D_COMPILE_STANDARD_FILE_INCLUDE, "PixelMain", "ps_5_0",
            shaderCompileFlags, 0U, debugPixelBytecode.GetAddressOf(),
            shaderErrors.GetAddressOf())))
        {
            if (shaderErrors)
                OutputDebugStringA(static_cast<const char*>(shaderErrors->GetBufferPointer()));
            return false;
        }
        if (FAILED(implementation_->device->CreateVertexShader(
                debugVertexBytecode->GetBufferPointer(), debugVertexBytecode->GetBufferSize(),
                nullptr, implementation_->debugVertexShader.GetAddressOf()))
            || FAILED(implementation_->device->CreatePixelShader(
                debugPixelBytecode->GetBufferPointer(), debugPixelBytecode->GetBufferSize(),
                nullptr, implementation_->debugPixelShader.GetAddressOf())))
            return false;

        const D3D11_INPUT_ELEMENT_DESC debugElements[]{
            {"POSITION", 0U, DXGI_FORMAT_R32G32B32_FLOAT, 0U,
                static_cast<UINT>(offsetof(DebugVertex, position)), D3D11_INPUT_PER_VERTEX_DATA, 0U},
            {"COLOR", 0U, DXGI_FORMAT_R32G32B32_FLOAT, 0U,
                static_cast<UINT>(offsetof(DebugVertex, color)), D3D11_INPUT_PER_VERTEX_DATA, 0U}};
        if (FAILED(implementation_->device->CreateInputLayout(debugElements, 2U,
            debugVertexBytecode->GetBufferPointer(), debugVertexBytecode->GetBufferSize(),
            implementation_->debugInputLayout.GetAddressOf())))
            return false;

        D3D11_BUFFER_DESC debugConstants{};
        debugConstants.ByteWidth = static_cast<UINT>(sizeof(Mat4));
        debugConstants.Usage = D3D11_USAGE_DYNAMIC;
        debugConstants.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
        debugConstants.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
        if (FAILED(implementation_->device->CreateBuffer(&debugConstants, nullptr,
            implementation_->debugViewProjectionBuffer.GetAddressOf())))
            return false;

        D3D11_DEPTH_STENCIL_DESC debugDepth{};
        debugDepth.DepthEnable = TRUE;
        debugDepth.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ZERO;
        debugDepth.DepthFunc = D3D11_COMPARISON_LESS_EQUAL;
        if (FAILED(implementation_->device->CreateDepthStencilState(
            &debugDepth, implementation_->debugDepthState.GetAddressOf())))
            return false;

        ComPtr<ID3D11Texture2D> backBuffer{};
        const HRESULT backBufferResult = implementation_->swapChain->GetBuffer(
            0U,
            IID_PPV_ARGS(backBuffer.GetAddressOf()));
        if(FAILED(backBufferResult))
        {
            return false;
        }

        const HRESULT renderTargetResult = implementation_->device->CreateRenderTargetView(backBuffer.Get(),nullptr,implementation_->renderTargetView.GetAddressOf());
        if(FAILED(renderTargetResult))
        {
            return false;
        }

        ID3D11RenderTargetView* renderTargets[] =
        {
            implementation_->renderTargetView.Get()
        };
        implementation_->deviceContext->OMSetRenderTargets(
            1U,
            renderTargets,
            implementation_->depthStencilView.Get());


        

        const D3D11_VIEWPORT viewport{
            0.0F,
            0.0F,
            static_cast<float>(width),
            static_cast<float>(height),
            0.0F,
            1.0F
        };
        implementation_->deviceContext->RSSetViewports(1U, &viewport);
        implementation_->cameraViewport = viewport;

        return true;
    }


    bool D3D11Renderer::Resize(std::uint32_t width, std::uint32_t height) noexcept
    {
        if (implementation_ == nullptr || implementation_->swapChain == nullptr
            || implementation_->device == nullptr || implementation_->deviceContext == nullptr
            || width == 0 || height == 0)
            return false;

        // Release the current back-buffer binding before DXGI resizes its buffers.
        implementation_->deviceContext->OMSetRenderTargets(0U, nullptr, nullptr);
        implementation_->renderTargetView.Reset();
        implementation_->depthStencilView.Reset();
        implementation_->depthStencilTexture.Reset();

        const HRESULT result = implementation_->swapChain->ResizeBuffers(
            0U, width, height, DXGI_FORMAT_UNKNOWN, 0U);

        if (FAILED(result))
            return false;

        ComPtr<ID3D11Texture2D> backBuffer{};

        const HRESULT bufferResult = implementation_->swapChain->GetBuffer(
            0U, IID_PPV_ARGS(backBuffer.GetAddressOf()));

        if (FAILED(bufferResult))
            return false;

        const HRESULT viewResult = implementation_->device->CreateRenderTargetView(
            backBuffer.Get(), nullptr, implementation_->renderTargetView.GetAddressOf());

        if (FAILED(viewResult))
            return false;

        D3D11_TEXTURE2D_DESC depthDescription{};
        depthDescription.Width = width;
        depthDescription.Height = height;
        depthDescription.MipLevels = 1U;
        depthDescription.ArraySize = 1U;
        depthDescription.Format = DXGI_FORMAT_D24_UNORM_S8_UINT;
        depthDescription.SampleDesc.Count = 1U;
        depthDescription.Usage = D3D11_USAGE_DEFAULT;
        depthDescription.BindFlags = D3D11_BIND_DEPTH_STENCIL;

        if (FAILED(implementation_->device->CreateTexture2D(
            &depthDescription, nullptr, implementation_->depthStencilTexture.GetAddressOf()))
            || FAILED(implementation_->device->CreateDepthStencilView(
                implementation_->depthStencilTexture.Get(), nullptr,
                implementation_->depthStencilView.GetAddressOf())))
        {
            return false;
        }

        ID3D11RenderTargetView* renderTargets[]{implementation_->renderTargetView.Get()};
        implementation_->deviceContext->OMSetRenderTargets(
            1U, renderTargets, implementation_->depthStencilView.Get());

        implementation_->aspectRatio = static_cast<float>(width) / static_cast<float>(height);

        const D3D11_VIEWPORT viewport{
            0.0F,
            0.0F,
            static_cast<float>(width),
            static_cast<float>(height),
            0.0F,
            1.0F
        };
        implementation_->deviceContext->RSSetViewports(1U, &viewport);
        implementation_->cameraViewport = viewport;

        return true;
    }

    bool D3D11Renderer::ClearFrame(
        float red,
        float green,
        float blue,
        float alpha) noexcept
    {
        if (implementation_ == nullptr || implementation_->deviceContext == nullptr
            || implementation_->swapChain == nullptr
            || implementation_->renderTargetView == nullptr)
        {
            return false;
        }

        const float clearColor[]{ red, green, blue, alpha };
        ID3D11RenderTargetView* renderTarget = implementation_->renderTargetView.Get();
        implementation_->deviceContext->OMSetRenderTargets(
            1U, &renderTarget, implementation_->depthStencilView.Get());
        implementation_->deviceContext->ClearRenderTargetView(
            renderTarget, clearColor);
        if (implementation_->depthStencilView != nullptr)
        {
            implementation_->deviceContext->ClearDepthStencilView(
                implementation_->depthStencilView.Get(),
                D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL,
                1.0F, 0U);
        }

        return SUCCEEDED(implementation_->swapChain->Present(1U, 0U));
    }

    bool D3D11Renderer::RenderFrame(float red, float green,float blue,float alpha, const Scene& scene,const AssetSystem& assets,
        const Camera& camera,
        std::span<const DebugLine> debugLines) noexcept
    {
        const Registry& registry = scene.GetRegistry();
        const Mat4& view = camera.ViewMatrix();
        const Mat4& projection = camera.ProjectionMatrix();

        const float clearColor[4] = {red, green, blue, alpha};
        implementation_->deviceContext->ClearRenderTargetView(implementation_->renderTargetView.Get(),clearColor);
        implementation_->deviceContext->ClearDepthStencilView( implementation_->depthStencilView.Get(),D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL,1.0f,0U);

        ID3D11ShaderResourceView* textureViews[] =
        {
            implementation_->diffuseTextureView.Get()
        };
        implementation_->deviceContext->PSSetShaderResources( 0U,1U,textureViews);

        ID3D11SamplerState* samplers[] =
        {
            implementation_->diffuseSampler.Get()
        };
        implementation_->deviceContext->PSSetSamplers(0U,1U,samplers);
        implementation_->deviceContext->PSSetSamplers(2U,1U,samplers);


        LightConstants lightConstants{};
        Mat4 lightViewProjection = Mat4::Identity();
        bool hasDirectionalLight = false;

        for (const Entity entity :
        registry.GetEntitiesWith<DirectionalLightComponent>())
        {
            const auto* light = registry.GetComponent<DirectionalLightComponent>(entity);

            const auto* lightTransform = registry.GetComponent<TransformComponent>(entity);

            if (light == nullptr || !light->enabled || lightTransform == nullptr)
            {
                continue;
            }

            const Vec3 directionToLight =lightTransform->rotation.Normalized().Rotate({ 0.0f, 0.0f, 1.0f }) * -1.0f;

            lightConstants.direction[0] = directionToLight.x;
            lightConstants.direction[1] = directionToLight.y;
            lightConstants.direction[2] = directionToLight.z;
            lightConstants.color[0] = std::max(0.0f, light->color.x);
            lightConstants.color[1] = std::max(0.0f, light->color.y);
            lightConstants.color[2] = std::max(0.0f, light->color.z);
            lightConstants.intensity = std::max(0.0f, light->intensity);

            const Vec3 sceneCenter{0.0f, 0.0f, 4.0f};
            const Vec3 lightEye = sceneCenter + directionToLight * 20.0f;
            const Mat4 lightView = Mat4::LookAt(lightEye, sceneCenter, {0.0f, 1.0f, 0.0f});
            const Mat4 lightProjection = Mat4::Orthographic(20.0f, 20.0f, 0.1f, 50.0f);
            lightViewProjection = lightView * lightProjection;
            hasDirectionalLight = true;

            break;
        }


        lightConstants.lightViewProjection = lightViewProjection;
        lightConstants.shadowsEnabled = hasDirectionalLight ? 1U : 0U;

        D3D11_MAPPED_SUBRESOURCE mappedLight{};
        if (FAILED(implementation_->deviceContext->Map(implementation_->lightConstantBuffer.Get(), 0U,D3D11_MAP_WRITE_DISCARD, 0U, &mappedLight)))
        {
            return false;
        }
        *static_cast<LightConstants*>(mappedLight.pData) = lightConstants;
        implementation_->deviceContext->Unmap(implementation_->lightConstantBuffer.Get(), 0U);

        CameraConstants cameraConstants{};
        cameraConstants.position[0] = camera.Position().x;
        cameraConstants.position[1] = camera.Position().y;
        cameraConstants.position[2] = camera.Position().z;
        D3D11_MAPPED_SUBRESOURCE mappedCamera{};
        if (FAILED(implementation_->deviceContext->Map(implementation_->cameraConstantBuffer.Get(), 0U, D3D11_MAP_WRITE_DISCARD, 0U, &mappedCamera)))
        {
            return false;
        }
        *static_cast<CameraConstants*>(mappedCamera.pData) = cameraConstants;
        implementation_->deviceContext->Unmap(implementation_->cameraConstantBuffer.Get(), 0U);

        const auto getAlbedoTextureView = [this](const std::string& path) -> ID3D11ShaderResourceView*
        {
            if (path.empty())
                return nullptr;

            const auto cached = implementation_->albedoTextureCache.find(path);
            if (cached != implementation_->albedoTextureCache.end())
                return cached->second.view.Get();

            Implementation::MaterialTexture texture{};
            ImageData image{};
            if (!LoadImageRgba8(path.c_str(), image))
            {
                OutputDebugStringA("QMEC could not load a material albedo texture.\n");
                implementation_->albedoTextureCache.emplace(path, std::move(texture));
                return nullptr;
            }

            D3D11_TEXTURE2D_DESC description{};
            description.Width = image.width;
            description.Height = image.height;
            description.MipLevels = 1U;
            description.ArraySize = 1U;
            description.Format = DXGI_FORMAT_R8G8B8A8_TYPELESS;
            description.SampleDesc.Count = 1U;
            description.Usage = D3D11_USAGE_IMMUTABLE;
            description.BindFlags = D3D11_BIND_SHADER_RESOURCE;

            D3D11_SUBRESOURCE_DATA initialData{};
            initialData.pSysMem = image.pixels.data();
            initialData.SysMemPitch = image.width * 4U;
            if (FAILED(implementation_->device->CreateTexture2D(&description, &initialData, texture.texture.GetAddressOf())))
            {
                implementation_->albedoTextureCache.emplace(path, std::move(texture));
                return nullptr;
            }

            D3D11_SHADER_RESOURCE_VIEW_DESC viewDescription{};
            viewDescription.Format = DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;
            viewDescription.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
            viewDescription.Texture2D.MostDetailedMip = 0U;
            viewDescription.Texture2D.MipLevels = 1U;
            if (FAILED(implementation_->device->CreateShaderResourceView(texture.texture.Get(), &viewDescription, texture.view.GetAddressOf())))
            {
                implementation_->albedoTextureCache.emplace(path, std::move(texture));
                return nullptr;
            }

            const auto [entry, wasInserted] = implementation_->albedoTextureCache.emplace(path, std::move(texture));
            (void)wasInserted;
            return entry->second.view.Get();
        };

        const auto getNormalTextureView = [this](const std::string& path) ->ID3D11ShaderResourceView*
            {
                if (path.empty())
                    return nullptr;

                const auto cached = implementation_->normalTextureCache.find(path);
                if (cached != implementation_->normalTextureCache.end())
                {
                    return cached->second.view.Get();
                }

                Implementation::MaterialTexture texture{};
                ImageData image{};
                if (!LoadImageRgba8(path.c_str(), image))
                {
                    OutputDebugStringA("QMEC could not load a material normal texture.\n");
                    return nullptr;
                }


                D3D11_TEXTURE2D_DESC description{};
                description.Width = image.width;
                description.Height = image.height;
                description.MipLevels = 1U;
                description.ArraySize = 1U;
                description.Format = DXGI_FORMAT_R8G8B8A8_TYPELESS;
                description.SampleDesc.Count = 1U;
                description.Usage = D3D11_USAGE_IMMUTABLE;
                description.BindFlags = D3D11_BIND_SHADER_RESOURCE;

                D3D11_SUBRESOURCE_DATA initialData{};
                initialData.pSysMem = image.pixels.data();
                initialData.SysMemPitch = image.width * 4U;
                if (FAILED(implementation_->device->CreateTexture2D(&description, &initialData, texture.texture.GetAddressOf())))
                {
                    return nullptr;
                }
                D3D11_SHADER_RESOURCE_VIEW_DESC viewDescription{};
                viewDescription.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
                viewDescription.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
                viewDescription.Texture2D.MostDetailedMip = 0U;
                viewDescription.Texture2D.MipLevels = 1U;
                if (FAILED(implementation_->device->CreateShaderResourceView(texture.texture.Get(), &viewDescription, texture.view.GetAddressOf())))
                {
                    return nullptr;
                }

                const auto [entry, wasInserted] = implementation_->normalTextureCache.emplace(path, std::move(texture));
                (void)wasInserted;
                return entry->second.view.Get();

            };

        const auto drawMeshes = [&](const Mat4& viewProjection, bool useMaterialShader) -> bool
        {
            
            implementation_->deviceContext->IASetInputLayout(implementation_->inputLayout.Get());
            implementation_->deviceContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
            implementation_->deviceContext->VSSetShader(implementation_->vertexShader.Get(), nullptr, 0U);
            ID3D11Buffer* meshConstants = implementation_->transformConstantBuffer.Get();
            implementation_->deviceContext->VSSetConstantBuffers(0U, 1U, &meshConstants);
            implementation_->deviceContext->OMSetDepthStencilState(nullptr, 0U);
            for (const Entity entity : registry.GetEntitiesWith<MeshRendererComponent>())
            {
                const auto* meshRenderer = registry.GetComponent<MeshRendererComponent>(entity);
                const auto* pose = registry.GetComponent<TransformComponent>(entity);

                if (meshRenderer == nullptr || !meshRenderer->visible || pose == nullptr)
                {
                    continue;
                }

                const GPUBuffer* mesh = assets.GetGPUMesh(meshRenderer->mesh);
                if (mesh == nullptr)
                {
                    continue;
                }


                TransformConstants transform{};
                transform.model = scene.GetWorldMatrix(entity);
                transform.modelViewProjection = transform.model * viewProjection;

                D3D11_MAPPED_SUBRESOURCE mapped{};
                if (FAILED(implementation_->deviceContext->Map(implementation_->transformConstantBuffer.Get(), 0U,
                    D3D11_MAP_WRITE_DISCARD, 0U, &mapped)))
                {
                    return false;
                }
                *static_cast<TransformConstants*>(mapped.pData) = transform;
                implementation_->deviceContext->Unmap(implementation_->transformConstantBuffer.Get(), 0U);

                if (useMaterialShader)
                {
                    const auto* material = registry.GetComponent<MaterialComponent>(entity);
                    MaterialConstants materialConstants{};
                    ID3D11ShaderResourceView* albedoTextureView = nullptr;
                    ID3D11ShaderResourceView* normalTextureView = nullptr;
                    if (material != nullptr)
                    {
                        materialConstants.albedoColor[0] = material->albedoColor.x;
                        materialConstants.albedoColor[1] = material->albedoColor.y;
                        materialConstants.albedoColor[2] = material->albedoColor.z;
                        materialConstants.albedoColor[3] = 1.0f;
                        materialConstants.normalStrength = material->normalStrength;
                        materialConstants.metallic = std::clamp(material->metallic, 0.0f, 1.0f);
                        materialConstants.roughness = std::clamp(material->roughness, 0.04f, 1.0f);
                        materialConstants.specularLevel = std::clamp(material->specularLevel, 0.0f, 1.0f);
                        albedoTextureView = getAlbedoTextureView(material->albedoTexturePath);
                        normalTextureView = getNormalTextureView(material->normalTexturePath);
                        materialConstants.hasAlbedoTexture = albedoTextureView != nullptr ? 1U : 0U;
                        materialConstants.hasNormalTexture = normalTextureView != nullptr ? 1U : 0U;
                    }

                    implementation_->deviceContext->PSSetShaderResources(2U, 1U, &albedoTextureView);
                    implementation_->deviceContext->PSSetShaderResources(3U, 1U, &normalTextureView);
                    D3D11_MAPPED_SUBRESOURCE mappedMaterial{};
                    if (FAILED(implementation_->deviceContext->Map(implementation_->materialConstantBuffer.Get(), 0U,
                        D3D11_MAP_WRITE_DISCARD, 0U, &mappedMaterial)))
                    {
                        return false;
                    }
                    *static_cast<MaterialConstants*>(mappedMaterial.pData) = materialConstants;
                    implementation_->deviceContext->Unmap(implementation_->materialConstantBuffer.Get(), 0U);
                    ID3D11Buffer* materialBuffer = implementation_->materialConstantBuffer.Get();
                    implementation_->deviceContext->PSSetConstantBuffers(2U, 1U, &materialBuffer);
                }

                ID3D11Buffer* vertexBuffer = mesh->vertexBuffer.Get();
                constexpr UINT stride = static_cast<UINT>(sizeof(Vertex));
                constexpr UINT offset = 0U;
                implementation_->deviceContext->IASetVertexBuffers(0U, 1U, &vertexBuffer, &stride, &offset);
                implementation_->deviceContext->IASetIndexBuffer(mesh->indexBuffer.Get(), DXGI_FORMAT_R32_UINT, 0U);
                implementation_->deviceContext->DrawIndexed(mesh->indexCount, 0U, 0);
            }
            return true;
        };

    
        ID3D11ShaderResourceView* noShadowResource = nullptr;
        implementation_->deviceContext->PSSetShaderResources(1U, 1U, &noShadowResource);
        implementation_->deviceContext->ClearDepthStencilView(implementation_->lightDepthStencilView.Get(), D3D11_CLEAR_DEPTH, 1.0f, 0U);

        bool shadowPassSucceeded = true;
        if (hasDirectionalLight)
        {
            const float shadowSize = static_cast<float>(implementation_->shadowMapResolution);
            const D3D11_VIEWPORT shadowViewport{0.0f, 0.0f, shadowSize, shadowSize, 0.0f, 1.0f};
            implementation_->deviceContext->RSSetViewports(1U, &shadowViewport);
            implementation_->deviceContext->OMSetRenderTargets(0U, nullptr, implementation_->lightDepthStencilView.Get());
          
            implementation_->deviceContext->PSSetShader(nullptr, nullptr, 0U);
            shadowPassSucceeded = drawMeshes(lightViewProjection, false);
        }

        
        ID3D11RenderTargetView* cameraTarget = implementation_->renderTargetView.Get();
        implementation_->deviceContext->OMSetRenderTargets(1U, &cameraTarget, implementation_->depthStencilView.Get());
        implementation_->deviceContext->RSSetViewports(1U, &implementation_->cameraViewport);
        implementation_->deviceContext->PSSetShader(implementation_->pixelShader.Get(), nullptr, 0U);

       
        ID3D11ShaderResourceView* shadowView = implementation_->lightTextureView.Get();
        implementation_->deviceContext->PSSetShaderResources(1U, 1U, &shadowView);
        ID3D11SamplerState* shadowSampler = implementation_->shadowSampler.Get();
        implementation_->deviceContext->PSSetSamplers(1U, 1U, &shadowSampler);

        if (!shadowPassSucceeded || !drawMeshes(view * projection, true))
        {
            return false;
        }

        if (!debugLines.empty())
        {
            constexpr UINT maxVertices = (std::numeric_limits<UINT>::max)() / static_cast<UINT>(sizeof(DebugVertex));
            if (debugLines.size() > maxVertices / 2U)
                return false;
            const UINT vertexCount = static_cast<UINT>(debugLines.size()) * 2U;

            if (vertexCount > implementation_->debugVertexCapacity)
            {
                const UINT grownCapacity = implementation_->debugVertexCapacity <= maxVertices / 2U? implementation_->debugVertexCapacity * 2U : maxVertices;
                const UINT capacity = (std::max)(vertexCount, grownCapacity);
                D3D11_BUFFER_DESC description{};
                description.ByteWidth = capacity * static_cast<UINT>(sizeof(DebugVertex));
                description.Usage = D3D11_USAGE_DYNAMIC;
                description.BindFlags = D3D11_BIND_VERTEX_BUFFER;
                description.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
                ComPtr<ID3D11Buffer> buffer{};
                if (FAILED(implementation_->device->CreateBuffer(
                    &description, nullptr, buffer.GetAddressOf())))
                    return false;
                implementation_->debugVertexBuffer = std::move(buffer);
                implementation_->debugVertexCapacity = capacity;
            }

            D3D11_MAPPED_SUBRESOURCE mappedVertices{};
            if (FAILED(implementation_->deviceContext->Map(implementation_->debugVertexBuffer.Get(), 0U, D3D11_MAP_WRITE_DISCARD,
                0U, &mappedVertices)))
                return false;
            auto* vertices = static_cast<DebugVertex*>(mappedVertices.pData);
            for (size_t i = 0; i < debugLines.size(); ++i)
            {
                vertices[i * 2U] = {debugLines[i].start, debugLines[i].color};
                vertices[i * 2U + 1U] = {debugLines[i].end, debugLines[i].color};
            }
            implementation_->deviceContext->Unmap(implementation_->debugVertexBuffer.Get(), 0U);

            D3D11_MAPPED_SUBRESOURCE mappedMatrix{};
            if (FAILED(implementation_->deviceContext->Map(implementation_->debugViewProjectionBuffer.Get(), 0U,D3D11_MAP_WRITE_DISCARD, 0U, &mappedMatrix)))
                return false;


            *static_cast<Mat4*>(mappedMatrix.pData) = view * projection;
            implementation_->deviceContext->Unmap(implementation_->debugViewProjectionBuffer.Get(), 0U);

            ID3D11Buffer* debugBuffer = implementation_->debugVertexBuffer.Get();
            constexpr UINT debugStride = static_cast<UINT>(sizeof(DebugVertex));
            constexpr UINT debugOffset = 0U;
            implementation_->deviceContext->IASetVertexBuffers(0U, 1U, &debugBuffer, &debugStride, &debugOffset);
            implementation_->deviceContext->IASetIndexBuffer(nullptr, DXGI_FORMAT_R32_UINT, 0U);
            implementation_->deviceContext->IASetInputLayout(implementation_->debugInputLayout.Get());
            implementation_->deviceContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST);
            implementation_->deviceContext->VSSetShader(implementation_->debugVertexShader.Get(), nullptr, 0U);
            implementation_->deviceContext->PSSetShader(implementation_->debugPixelShader.Get(), nullptr, 0U);
            ID3D11Buffer* debugConstants = implementation_->debugViewProjectionBuffer.Get();
            implementation_->deviceContext->VSSetConstantBuffers(0U, 1U, &debugConstants);
            implementation_->deviceContext->OMSetDepthStencilState(implementation_->debugDepthState.Get(), 0U);
            implementation_->deviceContext->Draw(vertexCount, 0U);
            implementation_->deviceContext->OMSetDepthStencilState(nullptr, 0U);
        }

        return SUCCEEDED(implementation_->swapChain->Present(1U, 0U));
    }

    bool D3D11Renderer::RenderFrame(float red, float green, float blue, float alpha,const Scene& scene, const AssetSystem& assets, Entity cameraEntity,
        std::span<const DebugLine> debugLines) noexcept
    {
        const Registry& registry = scene.GetRegistry();
        const auto* cameraComponent = registry.GetComponent<CameraComponent>(cameraEntity);
        const auto* cameraTransform = registry.GetComponent<TransformComponent>(cameraEntity);
        if (cameraComponent == nullptr || cameraTransform == nullptr)
        {
            return false;
        }

        Camera sceneCamera{};
        sceneCamera.SetPosition(cameraTransform->position);
        sceneCamera.SetOrientation(cameraTransform->rotation);
        sceneCamera.SetFov(cameraComponent->verticalFieldOfViewRadians);
        sceneCamera.SetNearPlane(cameraComponent->nearPlane);
        sceneCamera.SetFarPlane(cameraComponent->farPlane);
        sceneCamera.SetAspectRatio(implementation_->aspectRatio);

        return RenderFrame(red, green, blue, alpha, scene, assets, sceneCamera, debugLines);
    }
}
