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

vsg::ref_ptr<ResourceUse> Node::addOutput(const vsg::ref_ptr<ResourceUse>& output)
{
    output->resource->producer = vsg::ref_ptr(this);
    return outputs.emplace_back(output);
}

void Node::prepareResources()
{
}

vsg::ref_ptr<ResourceUse> Node::findResource(const vsg::ref_ptr<Resource> &resource)
{
    auto pred = [&resource](const auto& resUse)
    {
        return resUse->resource == resource;
    };
    auto inputItr = std::find_if(inputs.begin(), inputs.end(), pred);
    if (inputItr != inputs.end())
    {
        return *inputItr;
    }
    auto outputItr = std::find_if(outputs.begin(), outputs.end(), pred);
    if (outputItr != outputs.end())
    {
        return *outputItr;
    }
    return {};
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
        
RenderDAG::RenderDAG()
{
}

void RenderDAG::computeEdges()
{
    edges.clear();
    for (auto entry : nodeMap)
    {
        const auto& node = entry.second;
        for (const auto& inputUse : node->inputs)
        {
            edges.emplace(inputUse->resource->producer, node);
        }
        for (const auto& outputUse : node->outputs)
        {
            if (auto ancestor = outputUse->resource->ancestor; ancestor)
            {
                edges.emplace(ancestor, node);
            }
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

    std::optional<uint32_t> maxAttachmentForType(const std::vector<vsg::ref_ptr<ResourceUse>>& resUses,
                                                 AttachmentType attType)
    {
        auto itr = std::max_element(resUses.begin(), resUses.end(),
                                    [attType](const auto& res1, const auto& res2)
                                    {
                                        auto attach2 = isAttachment(res2, attType);
                                        if (!attach2)
                                        {
                                            return false;
                                        }
                                        auto attach1 = isAttachment(res1, attType);
                                        if (!attach1)
                                        {
                                            return true;
                                        }
                                        return attach1->attachment < attach2->attachment;
                                    });
        if (itr == resUses.end())
        {
            return {};
        }
        if (auto attachment = isAttachment(*itr, attType); attachment.valid())
        {
            return attachment->attachment;
        }
        return {};
    }
}

// Allocate the subpass description arrays. Attachment references need to be stored in specific
// elements.


void RenderNode::allocatePassAttachments()
{

    auto maxInputAttachment = maxAttachmentForType(inputs, Input);
    auto maxColorAttachment = maxAttachmentForType(outputs, Color);
    auto maxResolveAttachment = maxAttachmentForType(outputs, MultisampleResolve);
    

    if (maxInputAttachment)
    {
        description.inputAttachments.resize(*maxInputAttachment + 1,
                                            {VK_ATTACHMENT_UNUSED, VK_IMAGE_LAYOUT_UNDEFINED});
    }
    if (maxColorAttachment)
    {
        description.colorAttachments.resize(*maxColorAttachment + 1,
                                            {VK_ATTACHMENT_UNUSED, VK_IMAGE_LAYOUT_UNDEFINED});
    }
    if (maxResolveAttachment)
    {
        if (*maxResolveAttachment > *maxColorAttachment)
        {
            vsg::fatal("Illegal resolve attachment reference.");
        }
        auto resolveSize = std::max(*maxColorAttachment, *maxResolveAttachment) + 1;
        description.resolveAttachments.resize(resolveSize, {VK_ATTACHMENT_UNUSED, VK_IMAGE_LAYOUT_UNDEFINED});
    }
}

void RenderNode::prepareResources()
{
    allocatePassAttachments();
    for (const auto& resUse : inputs)
    {
        if (auto att = isAttachment(resUse, Input); att.valid())
        {
            setAttachmentUsage(att, VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT);
            auto attachIdx = addAttachment(att);
            auto passLayout = VK_IMAGE_LAYOUT_READ_ONLY_OPTIMAL;
            description.inputAttachments[att->attachment].attachment = attachIdx;
            description.inputAttachments[att->attachment].layout = passLayout;
            attachments[attachIdx].desiredInitialLayout = passLayout;
            attachments[attachIdx].attachment.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
        }
        else
        {
            // XXX Input texture: VK_IMAGE_USAGE_SAMPLED_BIT
        }
    }
    for (const auto& resUse : outputs)
    {
        vsg::ref_ptr<AttachmentUse> att;
        if ((att = isAttachment(resUse, Color)))
        {
            setAttachmentUsage(att, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT);
            description.colorAttachments[att->attachment].attachment = addAttachment(att);
            description.colorAttachments[att->attachment].layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        }
        else if ((att = isAttachment(resUse, Depth)))
        {
            setAttachmentUsage(att, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT);
            description.depthStencilAttachments
                .emplace_back(addAttachment(att), VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL);
        }
        else if ((att = isAttachment(resUse, MultisampleResolve)))
        {
            // Order must correspond to color attachments
            setAttachmentUsage(att, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT);
            description.resolveAttachments[att->attachment].attachment = addAttachment(att);
            description.resolveAttachments[att->attachment].layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        }
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


void RenderDAG::build(vsg::Device* device)
{
    topological_sort(nodes.size(), edges, sortedNodes);
    allocateTextures(device);
    // make RenderPass objects
    // make framebuffers
    // make rendergraphs
    // make commandGraph
}
