#pragma once

// A VSG take on the render graph implementation from Mastering Graphics

#include <vsg/core/Inherit.h>
#include <vsg/state/ImageView.h>
#include <vsg/state/Sampler.h>
#include <vsg/vk/RenderPass.h>

#include <limits>
#include <optional>

// From vsgCs
template<typename TSubclass, typename TParent>
vsg::ref_ptr<TSubclass> ref_ptr_cast(const vsg::ref_ptr<TParent>& p)
{
    return vsg::ref_ptr<TSubclass>(dynamic_cast<TSubclass*>(p.get()));
}

/*
 * @brief Types of the inputs and outputs of a node
 */
enum FrameGraphResourceType
{
  Invalid = -1,
  Buffer = 0,
  Texture = 1,
  Attachment = 2
};

enum AttachmentType
{
    Color = 0,
    Input,
    Depth,
    MultisampleResolve
};

// An operation, which can be either load or store
enum RenderPassOperation
{
    DontCare,
    Load,
    Clear
};

struct ImageDimensions
{
    enum Type
    {
        Absolute,
        FBScaled
    };
    Type dimType = FBScaled;
    float width = 1.0f;
    float height = 1.0f;
};

template <typename TV>
uint32_t lastIndex(const TV& vec)
{
    return static_cast<uint32_t>(&vec.back() - vec.data());
}


// These classes are the nodes and edges of the DAG. They contain the
// description, bookkeeping for sorting the graph, and eventually VSG objects
// that will instantiate the CommandGraph.

class Node;

const std::string& name(const vsg::ref_ptr<vsg::Object> &object);
void setName(const vsg::ref_ptr<vsg::Object> &object, const std::string &name);

class Resource : public vsg::Inherit<vsg::Object, Resource>
{
public:
    // Node that has most recently used this resource as output
    vsg::observer_ptr<Node> producer;
    // flags that describe the creation of the resource
    VkPipelineStageFlags srcStageMask;
    VkAccessFlags srcAccessMask;
    // For read/modify/write resources, the output resource that provided the initial input. There will be an
    // edge between that resource's producer and the node that contains this resource as an output.
    vsg::observer_ptr<Resource> ancestor;
    // Do we need to track a resource's layout through the graph?
};

template<typename TResource>
vsg::ref_ptr<TResource> RMW(const vsg::ref_ptr<TResource>& src, const std::string& instanceName)
{
    auto result = TResource::create(src);
    result->ancestor = src;
    setName(result, instanceName);
    return result;
}

// An ImageResource can be either an Attachment or Texture input. It can only be
// Attachment output until we get storage textures, buffers, etc.
//
// For input / specification, it makes sense to specify the creation of an image within a
// resource. Does it for implementation? One good reason: having to recreate the image during a resize.
class ImageResource : public vsg::Inherit<Resource, ImageResource>
{
public:
    ImageResource()
    {
    }
    ImageResource(const vsg::ref_ptr<vsg::Image>& in_source,
                  const vsg::ref_ptr<vsg::ImageView>& in_use)
        : source(in_source), use(in_use)
    {
    }
    vsg::ref_ptr<vsg::Image> source;
    vsg::ref_ptr<vsg::ImageView> use;
    ImageDimensions dimensions;
};

class PresentationResource
    : public vsg::Inherit<ImageResource, PresentationResource>
{
  
};

struct ResourceUse : public vsg::Inherit<vsg::Object, ResourceUse>
                         
{
    ResourceUse()
    {}
    ResourceUse(const vsg::ref_ptr<Resource>& in_resource)
        : resource(in_resource)
    {
    }

    vsg::ref_ptr<Resource> resource;
};

struct AttachmentUse : public vsg::Inherit<ResourceUse, AttachmentUse>
{
    AttachmentUse()
        : attachmentType(Color)
    {}
    AttachmentUse(const vsg::ref_ptr<Resource>& in_resource,
                  AttachmentType in_attachmentType = Color)
        : Inherit(in_resource), attachment(0), attachmentType(in_attachmentType),
          loadOp(VK_ATTACHMENT_LOAD_OP_DONT_CARE), storeOp(VK_ATTACHMENT_STORE_OP_DONT_CARE)
    {
    }
    uint32_t attachment;
    AttachmentType attachmentType;
    VkAttachmentLoadOp loadOp;
    VkAttachmentStoreOp storeOp;
};

inline vsg::ref_ptr<AttachmentUse> isAttachment(const vsg::ref_ptr<ResourceUse>& use, AttachmentType atype)
{
    if (auto attachment = ref_ptr_cast<AttachmentUse>(use); attachment.valid())
    {
        if (attachment->attachmentType == atype)
        {
            return attachment;
        }
    }
    return {};
}



class Node : public vsg::Inherit<vsg::Object, Node>
{
public:
    
    bool enabled = true;
    vsg::ref_ptr<ResourceUse> addInput(const vsg::ref_ptr<ResourceUse>& input);
    vsg::ref_ptr<ResourceUse> addOutput(const vsg::ref_ptr<ResourceUse>& output);
    // vsg::RenderGraph
    std::vector<vsg::ref_ptr<ResourceUse>> inputs;
    std::vector<vsg::ref_ptr<ResourceUse>> outputs;
    virtual void prepareResources();
    vsg::ref_ptr<ResourceUse> findResource(const vsg::ref_ptr<Resource>& resource);
    virtual VkImageLayout compareSetFinalLayout(const vsg::ref_ptr<Resource>& resource, VkImageLayout layout);
};

// Parameters needed to create an attachment in a render pass and in a frame buffer
struct NodeAttachment
{
    NodeAttachment(vsg::ref_ptr<AttachmentUse> in_resource = {});
    vsg::ref_ptr<AttachmentUse> resource;
    vsg::AttachmentDescription attachment;
    VkImageLayout desiredInitialLayout;
};

class RenderNode : public vsg::Inherit<Node, RenderNode>
{
public:
    // Are scaled dimensions needed here?
    float resolution_scale_width  = 0.f;
    float resolution_scale_height = 0.f;
    std::vector<NodeAttachment> attachments;
    uint32_t addAttachment(const vsg::ref_ptr<AttachmentUse>& att);
    // This is a convenient description of a pass' dependencies. When we optimze passes into
    // subpasses, I guess we'll have an array of these.
    vsg::SubpassDescription description;
    void prepareResources() override;
    VkImageLayout compareSetFinalLayout(const vsg::ref_ptr<Resource>& resource,
                                        VkImageLayout layout) override;
protected:
    void allocatePassAttachments();
    
};

// The RenderDAG creates a graph, and then a sorted order, from the input list of nodes and resources

template <typename TObject> struct std::hash<vsg::ref_ptr<TObject>>
{

    std::size_t operator()(vsg::ref_ptr<TObject> const& node)
    {
        return std::hash<uintptr_t>{}(reinterpret_cast<uintptr_t>(node.get()));
    }
};

class RenderDAG : public vsg::Inherit<vsg::Object, RenderDAG>
{
public:
    RenderDAG();
    // Dimensions that are the basis for the scaling parameters of resources. Need to handle
    // updating all those after a resize
    VkExtent2D dimensions;
    void computeEdges();
    void build(vsg::Device* device);
    vsg::CommandGraph getCommandGraph();
    vsg::RenderGraph getRenderGraph(const std::string& name);
    bool addResource(const vsg::ref_ptr<Resource>& resource, const std::string& name = "");
    bool addNode(const vsg::ref_ptr<Node>& node, const std::string& name = "");
    static const VkSampleCountFlags UseGraphSampleCount = VK_SAMPLE_COUNT_FLAG_BITS_MAX_ENUM;
    static vsg::ref_ptr<vsg::Image> makeImage(VkFormat format,
                                              VkSampleCountFlags sampleCount = UseGraphSampleCount);
    using Graph = std::unordered_multimap<vsg::ref_ptr<Node>, vsg::ref_ptr<Node>>;
protected:
    vsg::ref_ptr<vsg::CommandGraph> _commandGraph;
    // map from resource name to resource
    std::unordered_map<std::string, vsg::ref_ptr<Resource>> resourceMap;

    std::unordered_map<std::string, vsg::ref_ptr<Node>> nodeMap;
    Graph edges;
    void allocateTextures(vsg::Device* device);
    std::vector<vsg::ref_ptr<Node>> sortedNodes;
    void makeRenderPasses();
};
