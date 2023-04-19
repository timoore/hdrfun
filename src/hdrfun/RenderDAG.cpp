#include "RenderDAG.h"
#include <vsg/io/Logger.h>
#include <vsg/state/Sampler.h>
#include <vsg/vk/RenderPass.h>

#include <stack>
#include <stdexcept>

const std::string &name(vsg::Object* object)
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

void setName(vsg::Object* object, const std::string &name)
{
    object->setValue("name", name);
}

void Resource::setBirth(Node *node)
{
    if (birth.valid())
    {
        vsg::fatal("Resource is written in more than one node");
    }
    birth = node;
}

void Node::prepareResources()
{
}

VkImageLayout Node::compareSetFinalLayout(const vsg::ref_ptr<Resource>&, VkImageLayout)
{
    return VK_IMAGE_LAYOUT_UNDEFINED;
}
    
// Notes:
// When we need a previous frame's resouce, we will need to allocate multiple
// resources and ping-pong.


bool RenderDAG::addResource(const vsg::ref_ptr<Resource>& resource, const std::string& in_name)
{
    const std::string& realName = (in_name == "") ? name(resource) : in_name;
    auto itr = resourceMap.insert({realName, resource});
    return itr.second;
}

vsg::ref_ptr<Resource> RenderDAG::_getResource(const std::string &name)
{
    auto itr = resourceMap.find(name);
    if (itr == resourceMap.end())
    {
        return {};
    }
    return itr->second;
}

RenderDAG::RenderDAG()
{
}

void RenderDAG::computeEdges()
{
    edges.clear();
    for (auto resourceEntry : resourceMap)
    {
        auto producer = resourceEntry.second->birth;
        for (auto consumer : resourceEntry.second->consumers)
        {
            edges.emplace(producer, consumer.first);
        }
    }
}

// Sorting code copied from somewhere...

namespace
{
    using Item = vsg::ref_ptr<Node>;
    
    void dfs(const RenderDAG::Graph& graph,
             std::unordered_map<Item, int>  color,
             const Item& node,
             const std::function<void(const Item&)> post_order_func)
    {
        std::stack<Item> nodes;
        nodes.push(node);
 
        while (!nodes.empty())
        {
            const Item from = nodes.top();
 
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
 
    void topological_sort(const std::vector<vsg::ref_ptr<Node>>& input, const RenderDAG::Graph& graph,
                          std::vector<vsg::ref_ptr<Node>>& result)
    {
        auto n = input.size();
        result.resize(n);
        std::unordered_map<vsg::ref_ptr<Node>, int> color;
        for (auto& node : input)
        {
            color[node] = 0;
        }
        int j = 0;
        auto post_order_func = [&result, &j](const vsg::ref_ptr<Node>& node) {
            result[j++] = node;
        };

        for (const auto& node : input)
        {
            if (color[node] == 0)
            {
                dfs(graph, color, node, post_order_func);
            }
        }
        reverse(begin(result), end(result));
    }
}


NodeAttachment::NodeAttachment(vsg::ref_ptr<AttachmentUse> in_resource)
    :resource(in_resource), desiredInitialLayout(VK_IMAGE_LAYOUT_UNDEFINED)
{
    // Fill a vsg::AttachmentDescription with valid values that are probably
    // useless.
    attachment.flags = 0;
    attachment.format = VK_FORMAT_UNDEFINED;
    attachment.samples = VK_SAMPLE_COUNT_1_BIT;
    attachment.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    attachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    attachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    attachment.finalLayout = VK_IMAGE_LAYOUT_UNDEFINED;
}

vsg::ref_ptr<ImageResource> RenderNode::setDepthStencilOutput(const std::string &name,
                                                              const ImageResource::Info &info)
{
    vsg::ref_ptr<RenderDAG> ref_graph = graph;
    auto resource = ImageResource::create(info);
    ref_graph->addResource(resource, name);
    resource->addImageUsage(VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT);
    resource->birth = this;
    depthStencilOutput = resource;
    return resource;
}

vsg::ref_ptr<ImageResource> RenderNode::setDepthStencilInput(const std::string &name)
{
    vsg::ref_ptr<RenderDAG> ref_graph = graph;
    auto resource = ref_graph->getResource<ImageResource>(name);
    resource->addImageUsage(VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT);
    resource->consumers[this]++;
    depthStencilInput = resource;
    return resource;
}

vsg::ref_ptr<ImageResource> RenderNode::addColorOutput(const std::string &name, const ImageResource::Info &info, uint32_t index, const std::string &input)
{
    vsg::ref_ptr<RenderDAG> ref_graph = graph;
    auto resource = ImageResource::create(info);
    ref_graph->addResource(resource, name);
    resource->addImageUsage(VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT);
    resource->birth = this;
    colorOutputs.emplace_back(AttachmentUse::create(resource, index));
    if (input != "")
    {
        auto inputResource = ref_graph->getResource<ImageResource>(input);
        colorInputs.emplace_back(inputResource);
        inputResource->addImageUsage(VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT);
        inputResource->consumers.emplace_back(this);
    }
    return resource;
}

vsg::ref_ptr<ImageResource> RenderNode::addResolveOutput(const std::string &name,
                                                         const ImageResource::Info &info,
                                                         uint32_t index)
{
    vsg::ref_ptr<RenderDAG> ref_graph = graph;
    auto resource = ImageResource::create(info);
    ref_graph->addResource(resource, name);
    resource->addImageUsage(VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT);
    resource->producers.emplace_back(this);
    resolveOutputs.emplace_back(AttachmentUse::create(resource, index));
    return resource;
}

vsg::ref_ptr<ImageResource> RenderNode::addAttachmentInput(const std::string &name, uint32_t index)
{
    vsg::ref_ptr<RenderDAG> ref_graph = graph;
    auto resource = ref_graph->getResource<ImageResource>(name);
    resource->addImageUsage(VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT);
    resource->consumers.emplace_back(this);
    colorInputs.emplace_back(AttachmentUse::create(resource, index));
    return resource;
}

uint32_t RenderNode::addAttachment(const vsg::ref_ptr<AttachmentUse>& att)
{
    attachments.push_back(att);
    if (auto presentation = ref_ptr_cast<PresentationResource>(att->resource); presentation.valid())
    {
        // XXX Get the actual format!!!
        attachments.back().attachment.format = VK_FORMAT_B8G8R8A8_SRGB;
    }
    else if (auto attachImage = ref_ptr_cast<ImageResource>(att->resource); attachImage.valid())
    {
        if (attachImage->source.valid())
        {
                attachments.back().attachment.format = attachImage->source->format;
        }
        else
        {
            vsg::fatal("Attachment doesn't have image resource");
        }
    }
    return lastIndex(attachments);
}

// XXX Set ImageView usage when (if) VSG supports it.
namespace
{
    void setAttachmentUsage(const vsg::ref_ptr<ResourceUse>& use, VkImageUsageFlags usageFlags)
    {
        if (auto attachment = ref_ptr_cast<AttachmentUse>(use); attachment.valid())
        {
            auto imageResource = ref_ptr_cast<ImageResource>(attachment->resource);
            if (!imageResource)
            {
                vsg::warn("resource ", name(attachment->resource), " is not an image.");
            }
            else
            {
                imageResource->source->usage |= usageFlags;
            }
        }
    }

    std::optional<uint32_t> maxAttachmentUsed(const std::vector<vsg::ref_ptr<AttachmentUse>>& attachments)
    {
        if (attachments.empty())
        {
            return {};
        }
        auto itr = std::max_element(attachments.begin(), attachments.end(),
                                    [](const auto& att1, const auto& att2)
                                    {
                                        return att1->attachment < att2->attachment;
                                    });
        return (*itr)->attachment;
    }

}

// Allocate the subpass description arrays. Attachment references need to be stored in specific
// elements.


void RenderNode::allocatePassAttachments()
{
    auto maxInputAttachment = maxAttachmentUsed(colorOutputs);
    auto maxColorAttachment = maxAttachmentUsed(inputAttachments);
    auto maxResolveAttachment = maxAttachmentUsed(resolveOutputs);
    const vsg::AttachmentReference initReference = {VK_ATTACHMENT_UNUSED, VK_IMAGE_LAYOUT_UNDEFINED};

    if (maxInputAttachment)
    {
        description.inputAttachments.resize(*maxInputAttachment + 1, initReference);
    }
    if (maxColorAttachment)
    {
        description.colorAttachments.resize(*maxColorAttachment + 1, initReference);
    }
    if (maxResolveAttachment)
    {
        if (*maxResolveAttachment > *maxColorAttachment)
        {
            vsg::fatal("Illegal resolve attachment reference.");
        }
        auto resolveSize = std::max(*maxColorAttachment, *maxResolveAttachment) + 1;
        description.resolveAttachments.resize(resolveSize, initReference);
    }
    bool hasDepth = depthStencilInput.valid() || depthStencilOutput.valid();
    if (hasDepth)
    {
        description.depthStencilAttachments.resize(1, initReference);
    }
    auto totalAttachments = description.inputAttachments.size()
        + description.colorAttachments.size() + description.resolveAttachments.size()
        + (hasDepth ? 1 : 0);
    passAttachments.resize(totalAttachments);
    uint32_t paIdx = 0;
    for (size_t i = 0; i < colorOutputs.size(); ++i)
    {
        auto& colorOutput = colorOutputs[i];
        if (!colorOutput)
        {
            continue;
        }
        passAttachments[paIdx].resource = colorOutput->resource;
        vsg::AttachmentDescription& desc = passAttachments[paIdx].attachment;
        desc.format = colorOutput->resource->info.prototype->format; // XXX protouse->format?
        desc.samples = colorOutput->resource->info.prototype->samples;
        desc.loadOp = colorInputs[i].valid() ? VK_ATTACHMENT_LOAD_OP_LOAD : VK_ATTACHMENT_LOAD_OP_CLEAR;
        desc.storeOp = VK_ATTACHMENT_STORE_OP_STORE; // XXX or DONT_CARE
        colorOutput->passAttachment = paIdx++;
    }
    for (auto& input : inputAttachments)
    {
        if (!input)
        {
            continue;
        }
        passAttachments[paIdx].resource = input->resource;
        vsg::AttachmentDescription& desc = passAttachments[paIdx].attachment;
        desc.format = input->resource->info.prototype->format; // XXX protouse->format?
        desc.samples = input->resource->info.prototype->samples;
        desc.loadOp =  VK_ATTACHMENT_LOAD_OP_LOAD;
        desc.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        input->passAttachment = paIdx++;
    }
    for (auto& resolve : resolveOutputs)
    {
        if (!resolve)
        {
            continue;
        }
        passAttachments[paIdx].resource = resolve->resource;
        vsg::AttachmentDescription& desc = passAttachments[paIdx].attachment;
        desc.format = resolve->resource->info.prototype->format; // XXX protouse->format?
        desc.samples = resolve->resource->info.prototype->samples; // Must be 1 sample?
        desc.loadOp =  VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        desc.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        resolve->passAttachment = paIdx++;
    }
    if (depthStencilOutput || depthStencilInput)
    {
        vsg::AttachmentDescription& desc = passAttachments[paIdx].attachment;
        if (depthStencilOutput)
        {
            passAttachments[paIdx].resource = depthStencilOutput->resource;
            desc.format = depthStencilOutput->resource->info.prototype->format;
            desc.samples = depthStencilOutput->resource->info.prototype->samples;
        }
        else
        {
            passAttachments[paIdx].resource = depthStencilInput->resource;
            desc.format = depthStencilInput->resource->info.prototype->format;
            desc.samples = depthStencilInput->resource->info.prototype->samples;

        }
        if (depthStencilInput)
        {
            if (depthStencilInput != depthStencilOutput)
            {
                vsg::fatal("Can't copy depth stencil image yet");
            }
            desc.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
            depthStencilInput->passAttachment = paIdx;
        }
        else
        {
            desc.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        }
        if (depthStencilOutput)
        {
            desc.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
            depthStencilOutput->passAttachment =paIdx;
        }
        else
        {
            desc.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        }
    }
}

void RenderNode::prepareResources()
{
    allocatePassAttachments();
    for (auto& input : inputAttachments)
    {
        input->resource->info.prototype->usage |= VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT;
        auto descriptionIdx = input->attachment;
        auto passAttachmentIdx = input->passAttachment;
        const auto passLayout = VK_IMAGE_LAYOUT_READ_ONLY_OPTIMAL;
        description.inputAttachments[descriptionIdx].attachment = passAttachmentIdx;
        description.inputAttachments[descriptionIdx].layout = passLayout;
        passAttachments[passAttachmentIdx].desiredInitialLayout = passLayout;
    }
    for (auto& colorOutput : colorOutputs)
    {
        colorOutput->resource->info.prototype->usage |= VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
        auto descriptionIdx = colorOutput->attachment;
        auto passAttachmentIdx = colorOutput->passAttachment;
        const auto passLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        description.colorAttachments[descriptionIdx].attachment = passAttachmentIdx;
        description.colorAttachments[descriptionIdx].layout = passLayout;
        passAttachments[passAttachmentIdx].desiredInitialLayout = passLayout;
    }
    for (auto& resolveOutput : resolveOutputs)
    {
        auto descriptionIdx = resolveOutput->attachment;
        if (description.colorAttachments[descriptionIdx].attachment == VK_ATTACHMENT_UNUSED)
        {
            vsg::fatal("resolve output doesn't have a corresponding color output");
        }
        auto passAttachmentIdx = resolveOutput->passAttachment;
        resolveOutput->resource->info.prototype->usage |= VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
        const auto passLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        description.resolveAttachments[descriptionIdx].attachment = passAttachmentIdx;
        description.resolveAttachments[descriptionIdx].layout = passLayout;
        passAttachments[passAttachmentIdx].desiredInitialLayout = passLayout;
    }

    auto depth = depthStencilOutput.valid() ? depthStencilOutput : depthStencilInput;
    if (depth)
    {
        depth->resource->info.prototype->usage |= VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
        auto passAttachmentIdx = depth->passAttachment;
        const auto passLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        description.depthStencilAttachments[0].attachment = passAttachmentIdx;
        description.depthStencilAttachments[0].layout = passLayout;
        passAttachments[passAttachmentIdx].desiredInitialLayout = passLayout;

    }

    // XXX An attachment could have more than one use, in a pass with aliasing; how to deal with
    // that?
    // Set the layout and store op in each resource's producer node
    for (auto& nodeAttachment : attachments)
    {
        auto resUse = nodeAttachment.resource;
        vsg::ref_ptr<Node> ref_producer = resUse->resource->producer;
        if (ref_producer.get() == this)
        {
            ref_producer = resUse->resource->ancestor;
        }
        if (ref_producer)
        {
            nodeAttachment.attachment.initialLayout
                = ref_producer->compareSetFinalLayout(resUse->resource, nodeAttachment.desiredInitialLayout);
            // set producer store op
        }
    }
    // subpass dependencies
}

VkImageLayout RenderNode::compareSetFinalLayout(const vsg::ref_ptr<Resource> &resource, VkImageLayout layout)
{
    auto itr = std::find(attachments.begin(), attachments.end(),
                         [&resource](const NodeAttachment& attachment)
                         {
                             return attachment.resource == resource;
                         });
    if (itr == attachments.end())
    {
        return VK_IMAGE_LAYOUT_UNDEFINED;
    }
    if (itr->attachment.finalLayout != VK_IMAGE_LAYOUT_UNDEFINED)
    {
        return itr->attachment.finalLayout;
    }
    itr->attachment.finalLayout = layout;
    return layout;
}

#if 0
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
#endif

void RenderDAG::makeRenderPasses()
{
    for (auto node : sortedNodes)
    {
        node->prepareResources();
    }

}

// Work backwards
// presentation resource has some kind of backing resource, and externals will
// too

void RenderDAG::allocateBackingResources()
{
  
}

void RenderDAG::build(vsg::Device* device)
{
    std::vector<vsg::ref_ptr<Node>> nodeScratch;
    std::transform(nodeMap.begin(), nodeMap.end(), std::back_inserter(nodeScratch),
        [](const auto& entry)
        {
            return entry.second;
        });
    topological_sort(nodeScratch, edges, sortedNodes);
    for (uint32_t i = 0; i < sortedNodes.size(); ++i)
    {
        sortedNodes[i]->passIndex = i;
    }
    makeRenderPasses();
    mergeResources();
    // allocateTextures(device);
    // make RenderPass objects
    // make framebuffers
    // make rendergraphs
    // make commandGraph
}
