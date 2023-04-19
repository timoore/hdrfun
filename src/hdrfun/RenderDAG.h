#pragma once

// A VSG take on the render graph implementation from Mastering Graphics and Maister's Graphics
// Adventures.

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
        FBScaled,
        ResourceRelative
    };
    Type dimType = FBScaled;
    float width = 1.0f;
    float height = 1.0f;
    std::optional<std::string> relativeTo;
};

constexpr const uint32_t UInitInvalid = std::numeric_limits<uint32_t>::max();

template <typename TV>
uint32_t lastIndex(const TV& vec)
{
    return static_cast<uint32_t>(&vec.back() - vec.data());
}

template <typename T>
T& resizeAt(std::vector<T>& vec, uint32_t index)
{
    if (index >= vec.size())
    {
        vec.resize(index + 1);
    }
    return vec[index];
}

// These classes are the nodes and edges of the DAG. They contain the
// description, bookkeeping for sorting the graph, and eventually VSG objects
// that will instantiate the CommandGraph.

class Node;
class RenderDAG;

const std::string& name(vsg::Object* object);
void setName(vsg::Object* object, const std::string &name);

// Resources are written to once and can be read multiple times (like SSA). Real
// resources are often read/modify/write, but that will be resolved when we
// assign the backing resources / do aliasing.

class Resource : public vsg::Inherit<vsg::Object, Resource>
{
public:
    Resource(const std::string& name = "")
    {
        setName(this, name);
    }
    vsg::observer_ptr<RenderDAG> graph;
    void addConsumer(Node* node)
    {

    }
    std::map<vsg::observer_ptr<Node>, uint32_t> consumers;
    // Node that creates this resource
    vsg::observer_ptr<Node> birth;
    void setBirth(Node* node);
    // Last node that uses the resource
        std::vector<vsg::observer_ptr<Node>> death;
    // flags that describe the creation of the resource
    VkPipelineStageFlags srcStageMask;
    VkAccessFlags srcAccessMask;
    // Do we need to track a resource's layout through the graph?
};

// An ImageResource can be either an Attachment or Texture input. It can only be
// Attachment output until we get storage textures, buffers, etc.
//
// For input / specification, it makes sense to specify the creation of an image
// within a resource. Does it for implementation? One good reason: having to
// recreate the image during a resize.

class ImageResource : public vsg::Inherit<Resource, ImageResource>
{
public:
    struct Info
    {
        vsg::ref_ptr<vsg::Image> prototype;
        vsg::ref_ptr<vsg::ImageView> protoUse;
        ImageDimensions dimensions;
    };
    ImageResource(const std::string& name, const Info& in_info)
        : Inherit(name), info(in_info), imageUsage(0)
    {
    }
    vsg::ref_ptr<vsg::Image> source;
    vsg::ref_ptr<vsg::ImageView> use;
    Info info;
    // Should this be set in info.prototype->usage?
    VkImageUsageFlags imageUsage;
    VkImageUsageFlags addImageUsage(VkImageUsageFlags usage)
    {
        return imageUsage |= usage;
    }
};

class PresentationResource
    : public vsg::Inherit<ImageResource, PresentationResource>
{
  
};

struct ResourceUse : public vsg::Inherit<vsg::Object, ResourceUse>
                         
{
    VkPipelineStageFlags stages = 0;
    VkAccessFlags access = 0;
    VkImageLayout layout = VK_IMAGE_LAYOUT_UNDEFINED;
};

struct ImageUse : public vsg::Inherit<ResourceUse, ImageUse>
{
    ImageUse(vsg::ref_ptr<ImageResource> in_resource = {})
        : resource(in_resource), passAttachment(UInitInvalid)
    {
    }
    vsg::ref_ptr<ImageResource> resource;
    // Number of the attachment in the pass' total list of attachments i.e., as used in the subpass
    // description.
    uint32_t passAttachment;
};

struct AttachmentUse : public vsg::Inherit<ImageUse, AttachmentUse>
{
    AttachmentUse()
    {}
    AttachmentUse(const vsg::ref_ptr<Resource>& in_resource, uint32_t in_attachment)
        : Inherit(in_resource), attachment(in_attachment)

    {
    }
    // number of the attachment in the subpass description, if any
    uint32_t attachment;
};


// A rendering pass
class Node : public vsg::Inherit<vsg::Object, Node>
{
public:
    Node(const std::string &name = "")
        : enabled(true), passIndex(UInitInvalid)
    {
        setName(this, name);
    }
    vsg::observer_ptr<RenderDAG> graph;
    bool enabled;
    // Index in sorted pass list
    uint32_t passIndex;
    // vsg::RenderGraph
    virtual void getInputs(std::vector<vsg::ref_ptr<Resource>>& inputs) = 0;
    virtual void getOutputs(std::vector<vsg::ref_ptr<Resource>>& outputs) = 0;
    virtual void prepareResources();
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
    vsg::ref_ptr<ImageResource> setDepthStencilOutput(const std::string &name,
                                                      const ImageResource::Info &info);
    vsg::ref_ptr<ImageResource> setDepthStencilInput(const std::string &name);
    vsg::ref_ptr<ImageResource> addColorOutput(const std::string &name,
                                               const ImageResource::Info &info,
                                               uint32_t index,
                                               const std::string &input = "");
    vsg::ref_ptr<ImageResource> addResolveOutput(const std::string &name,
                                                 const ImageResource::Info &info,
                                                 uint32_t index);
    vsg::ref_ptr<ImageResource> addAttachmentInput(const std::string &name,
                                                   uint32_t index);

    uint32_t addAttachment(const vsg::ref_ptr<AttachmentUse>& att);
    // This is a convenient description of a pass' dependencies. When we optimze passes into
    // subpasses, I guess we'll have an array of these.
    vsg::SubpassDescription description;
    void prepareResources() override;
    VkImageLayout compareSetFinalLayout(const vsg::ref_ptr<Resource>& resource,
                                        VkImageLayout layout) override;

    std::vector<vsg::ref_ptr<AttachmentUse>> colorOutputs;
    std::vector<vsg::ref_ptr<ImageUse>> colorInputs;
    std::vector<vsg::ref_ptr<AttachmentUse>> inputAttachments;
    std::vector<vsg::ref_ptr<AttachmentUse>> resolveOutputs;
    vsg::ref_ptr<ImageUse> depthStencilInput;
    vsg::ref_ptr<ImageUse> depthStencilOutput;

    std::vector<NodeAttachment> passAttachments;
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
    template<typename TRresource>
    vsg::ref_ptr<TRresource> getResource(const std::string& name)
    {
        return ref_ptr_cast<TRresource>(_getResource(name));
    }
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
    void allocateBackingResources();
    vsg::ref_ptr<Resource> _getResource(const std::string& name);

};
