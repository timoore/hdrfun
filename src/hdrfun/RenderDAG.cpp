#include "RenderDAG.h"
#include <vsg/io/Logger.h>
#include <vsg/state/Sampler.h>
#include <vsg/vk/RenderPass.h>

#include <stack>
#include <stdexcept>

const std::string &name(const vsg::ref_ptr<vsg::Object> &object)
{
    vsg::Object* obj = object->getObject("name");
    if (!obj)
    {
        throw std::logic_error("object must have a name");
    }
    vsg::Value<std::string>* value = dynamic_cast<vsg::Value<std::string>*>(obj);
    if (!value)
    {
        throw std::logic_error("object must have a name");
    }
    return value->value();
}

void setName(const vsg::ref_ptr<vsg::Object> &object, const std::string &name)
{
    object->setValue("name", name);
}

vsg::ref_ptr<ResourceUse> Node::addInput(const vsg::ref_ptr<ResourceUse>& input)
{
    return inputs.emplace_back(input);
}

vsg::ref_ptr<ResourceUse> Node::addWriteOnlyOutput(const vsg::ref_ptr<ResourceUse>& output)
{
    output->resource->producer = vsg::ref_ptr(this);
    return outputs.emplace_back(output);
}

vsg::ref_ptr<ResourceUse> Node::addReadWriteOutput(const vsg::ref_ptr<Resource>& input,
                                                 const std::string& outputName)
{
    auto inputCopy = Resource::create(input);
    setName(inputCopy, outputName);
    inputCopy->ancestor = input;
    inputCopy->producer = vsg::ref_ptr(this);
    return outputs.emplace_back(inputCopy);
}

// Notes:
// When we need a previous frame's resouce, we will need to allocate multiple
// resources and ping-pong.

#if 0
Resource& RenderDAG::addResource(const vsg::ref_ptr<ResourceDesc> &desc,
                            bool isOutput)
{
    auto itr = resourceMap.find(desc->name());
    if (itr == resourceMap.end())
    {
        resources.emplace_back(desc);
        auto insertResult = resourceMap.insert({desc->name(), backIndex(resources)});
        itr = insertResult.first;
    }
    if (isOutput)
    {
        if ()
        resRef.outputHandle = backIndex(resources);
    }
    return resRef;
}
#endif

bool RenderDAG::addResource(const vsg::ref_ptr<Resource>& resource, const std::string& in_name)
{
    const std::string& realName = (in_name == "") ? name(resource) : in_name;
    auto itr = resourceMap.insert({realName, resource});
    return itr.second;
}
        
RenderDAG::RenderDAG()
{
    for (const auto& nodeDesc : nodesDescs)
    {
        auto& nodeRef = nodes.emplace_back(nodeDesc);
        auto nodeHandle = &nodeRef - nodes.data();
        auto insrt = nodeMap.insert({nodeDesc->name(), nodeHandle});
        if (!insrt.second)
        {
            vsg::fatal("multiple nodes named ", nodeDesc->name());
        }
        for (const auto& resDesc : nodeDesc->inputs())
        {
            addResource(resDesc, false);
        }
        for (const auto& resDesc : nodeDesc->outputs())
        {
            auto& resource = addResource(resDesc, true);
            resource.producer = nodeHandle;
        }
    }
    for (const auto& node : nodes)
    {
        for (auto resHandle : node.outputs)
        {
            auto& res = resources[resHandle];
            if (!res.desc->loadFrom().empty())
            {
                auto itr = resourceMap.find(res.desc->loadFrom());
                if (itr == resourceMap.end())
                {
                    vsg::fatal("No resource named ", res.desc->loadFrom());
                }
                res.loadFromHandle = itr->second;
            }
        }
    }
}

void RenderDAG::computeEdges()
{
    edges.clear();
    for (size_t i = 0; i < nodes.size(); ++i)
    {
        auto& node = nodes[i];
        for (auto input : node.inputs)
        {
            auto& inputRes = resources[input];
            if (inputRes.outputHandle == invalidHandle&& !inputRes.desc->creationDesc()->external())
            {
                vsg::fatal("Resource ", inputRes.desc->name(), "in node ",
                           node.desc->name(), "has no producer");
            }
            auto& outputRes = resources[inputRes.outputHandle];
            inputRes.producer = outputRes.producer;
            inputRes.outputHandle = outputRes.outputHandle;
            edges.emplace(inputRes.producer, i);
        }
    }
}

// Sorting code copied from somewhere...

namespace
{
    void dfs(const std::unordered_multimap<NodeHandle, NodeHandle>& graph,
             std::vector<int>& color,
             NodeHandle node,
             const std::function<void(NodeHandle)> post_order_func)
    {
        std::stack<int> nodes;
        nodes.push(node);
 
        while (!nodes.empty())
        {
            int from = nodes.top();
 
            if (color[from] == 1)
            {
                color[from] = 2;
                post_order_func(from);
                nodes.pop();
                continue;
            }
            else if (color[from] == 2)
            {
                nodes.pop();
                continue;
            }
 
            color[from] = 1;
            auto range = graph.equal_range(from);
 
            for (auto it = range.first; it != range.second; ++it)
            {
                const auto& to = it->second;
                if (color[to] == 0)
                {
                    nodes.push(to);
                }
                else if (color[to] == 1)
                {
                    throw std::runtime_error("Graph has cycle. Topological sort impossible.");
                }
            }
        }
    }
 
    void topological_sort(int n, const std::unordered_multimap<NodeHandle, NodeHandle>& graph,
                          std::vector<NodeHandle>& result)
    {
        result.resize(n);
        std::vector<int> color(n, 0);
        int j = 0;
        auto post_order_func = [&result, &j](int node) {
            result[j++] = node;
        };
 
        for (int i = 0; i < n; ++i)
        {
            if (color[i] == 0)
            {
                dfs(graph, color, i, post_order_func);
            }
        }
 
        reverse(begin(result), end(result));
    }
}

void RenderDAG::setInputUsage(ResourceHandle resHandle) 
{
    auto& inputRes = resources[resHandle];
    if (inputRes.resType == Texture || inputRes.resType == Attachment)
    {
        auto& outputRes = resources[inputRes.producer];
        vsg::ref_ptr<TextureDesc> texDesc = outputRes.desc->creationDesc().cast<TextureDesc>();
        if (texDesc)
        {
            texDesc->usage() |= (inputRes.resType == Texture
                                 ? VK_IMAGE_USAGE_SAMPLED_BIT : VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT);
        }
    }
}

void RenderDAG::setOutputUsage(ResourceHandle resHandle)
{
    auto& outputRes = resources[resHandle];
    vsg::ref_ptr<TextureDesc> texDesc = outputRes.desc->creationDesc().cast<TextureDesc>();
    if (texDesc)
    {
        // XXX Better way to determine depth usage?
        if (texDesc->format() == VK_FORMAT_D32_SFLOAT)
        {
            texDesc->usage() |= VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
        }
        else
        {
            texDesc->usage() |= VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;            
        }
    }
}

namespace
{
    void createImageView(Resource& outputRes, vsg::Device* device, VkExtent2D& extent)
    {
        VkExtent3D imageExtent{0, 0, 1};
        vsg::ref_ptr<TextureDesc> texDesc = outputRes.desc->creationDesc().cast<TextureDesc>();
        if (!texDesc)
        {
            vsg::fatal("Resource ", outputRes.desc->name(), "has no texture description");
        }
        if (texDesc->width() != 0)
        {
            imageExtent.width = texDesc->width();
        }
        else
        {
            imageExtent.width = texDesc->scale().x * extent.width;
        }
        if (texDesc->height() != 0)
        {
            imageExtent.height = texDesc->height();
        }
        else
        {
            imageExtent.height = texDesc->scale().y * extent.height;
        }
        auto image = vsg::Image::create();
        image->imageType = VK_IMAGE_TYPE_2D;
        image->format = texDesc->format();
        image->extent = imageExtent;
        image->mipLevels = 1;
        image->arrayLayers = 1;
        image->samples = VK_SAMPLE_COUNT_1_BIT;
        image->tiling = VK_IMAGE_TILING_OPTIMAL;
        image->usage = texDesc->usage();
        image->initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        image->flags = 0;
        image->sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        auto aspectFlags = vsg::computeAspectFlagsForFormat(image->format);
        texDesc->imageView = createImageView(device, image, aspectFlags);

        // Sampler for accessing attachment as a texture
        if ((aspectFlags & VK_IMAGE_ASPECT_COLOR_BIT))
        {
            texDesc->sampler = vsg::Sampler::create();
            texDesc->sampler->flags = 0;
            texDesc->sampler->magFilter = VK_FILTER_LINEAR;
            texDesc->sampler->minFilter = VK_FILTER_LINEAR;
            texDesc->sampler->mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
            texDesc->sampler->addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
            texDesc->sampler->addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
            texDesc->sampler->addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
            texDesc->sampler->mipLodBias = 0.0f;
            texDesc->sampler->maxAnisotropy = 1.0f;
            texDesc->sampler->minLod = 0.0f;
            texDesc->sampler->maxLod = 1.0f;
            texDesc->sampler->borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;
        }
    }
}

void RenderDAG::allocateTextures(vsg::Device* device)
{

    for (auto nodeHandle : sortedNodes)
    {
        auto& node = nodes[nodeHandle];
        for (auto inputResHandle : node.inputs)
        {
            setInputUsage(inputResHandle);
        }
        for (auto outputResHandle : node.outputs)
        {
            setOutputUsage(outputResHandle);
        }
    }
    for (auto nodeHandle : sortedNodes)
    {
        auto& node = nodes[nodeHandle];
        for (auto resHandle : node.outputs)
        {
            auto& resource = resources[resHandle];
            if (resource.resType == Attachment)
            {
                createImageView(resource, device, dimensions);
            }
        }
    }
}

void RenderDAG::makeRenderPasses()
{
    for (auto nodeHandle : sortedNodes)
    {
        auto& node = nodes[nodeHandle];
        std::vector<ResourceHandle> attachments;
        vsg::RenderPass::Attachments vsgAttachments;
        for (auto outputResHandle : node.outputs)
        {
            auto& outputRes = resources[outputResHandle];
            if (outputRes.resType == Attachment)
            {
                // Other kinds of outputs?
                attachments.push_back(outputRes.outputHandle);
                auto& attach = vsgAttachments.emplace_back();
                // attach.format = outputRes.desc->creationDesc()->

                
            }
        }
        for (auto inputResHandle : node.inputs)
        {
            auto& inputRes = resources[inputResHandle];
            if (inputRes.resType == Attachment)
            {
                attachments.push_back(inputRes.outputHandle);
            }
        }

    }
}

// Many fields can (should) be deferred until the render graph is compiled.
vsg::ref_ptr<vsg::Image> RenderDAG::makeImage(VkFormat format, VkSampleCountFlags sampleCount)
{
    auto image = vsg::Image::create();
    image->imageType = VK_IMAGE_TYPE_2D;
    image->format = format;
    image->mipLevels = 1;
    image->arrayLayers = 1;
    image->samples = sampleCount;
    image->tiling = VK_IMAGE_TILING_OPTIMAL;
//    image->usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT;
    image->initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    image->flags = 0;
    image->sharingMode = VK_SHARING_MODE_EXCLUSIVE;
}

void RenderDAG::build(vsg::Device* device)
{
    topological_sort(nodes.size(), edges, sortedNodes);
    allocateTextures(device);
    // make RenderPass objects
    // make framebuffers
    // make rendergraphs
    // make commandGraph
}
