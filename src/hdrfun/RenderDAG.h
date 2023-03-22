#pragma once

// A VSG take on the render graph implementation from Mastering Graphics

#include <vsg/core/Inherit.h>
#include <vsg/state/ImageView.h>
#include <vsg/state/Sampler.h>

#include <limits>

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
    Output = 0,
    Input,
    Depth,
    MultisampleResolve,
    Presentation                // Denotes the eventual destination of rendering
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

class FrameGraphNode;

template <typename T>
uint32_t backIndex(const T& vector)
{
    return static_cast<uint32_t>(&vector.back() - vector.data());
}

// The Desc classes are holders of data about render passes and resources. The funky member
// definitions all a poor man's C / C++ 2020 named member / fluent style.
// are relatively easy to initalize by hand.

// Macro arguments can't contain literal commas!
#define COMMA ,
#define MEMBER(TYPE, NAME, INIT) TYPE& NAME() { return NAME ## _; }     \
    auto NAME(const TYPE& val) { NAME ## _ = val; return *this; } \
    TYPE NAME ## _ = INIT;

#if 0
class TextureDesc : public vsg::Inherit<CreationDesc, TextureDesc>
{
public:
    MEMBER(uint32_t, width, 0);
    MEMBER(uint32_t, height, 0);
    MEMBER(uint32_t, depth, 1);
    MEMBER(vsg::vec2, scale, {1.0f COMMA 1.0f});
    MEMBER(VkFormat, format, VK_FORMAT_UNDEFINED);
    MEMBER(VkImageUsageFlags, usage, 0);
    MEMBER(RenderPassOperation, loadOp, DontCare);
    MEMBER(vsg::vec4, clearColor, {0.0f COMMA 0.0f COMMA 0.0f COMMA 1.0f});
    MEMBER(VkClearDepthStencilValue, clearDepthStencil, {0.0f COMMA 0});

    // XXX Should this have a value for the presentation ImageViews?
    vsg::ref_ptr<vsg::ImageView> imageView;
    vsg::ref_ptr<vsg::Sampler> sampler;
    vsg::ref_ptr<TextureDesc> refptr() { return vsg::ref_ptr(this); }
};

// The name of a resource is actually the unique name of the resource as an edge
// between two nodes. In order for to share read-modify-write resource among
// several nodes, a "loadFrom" member specifies the source object of such an
// output resource.

class ResourceDesc : public vsg::Inherit<vsg::Object, ResourceDesc>
{
    public:
    MEMBER(vsg::ref_ptr<CreationDesc>, creationDesc, {});
    MEMBER(FrameGraphResourceType, resType, Invalid);
    vsg::observer_ptr<FrameGraphNode> producer;
    int refCount;               // needed?
    MEMBER(std::string, name, "");
    MEMBER(std::string, loadFrom, "");
    vsg::ref_ptr<TextureDesc> refptr() { return vsg::ref_ptr(this); }
};

// Our graph node will contain a RenderGraph, which encapsulates a frame buffer,
// its render pass, and a view which contains the scene graph to render.
//
// Why not call it a PassDesc? It could be something other than a RenderPass
// e.g., compute.
//
// inputs holds the read-only inputs to a node. Input resources of type
// Attachment will be renderpass input attachments.
//
// outputs holds read-modify-write and write-only resources.

class NodeDesc : public vsg::Inherit<vsg::Object, NodeDesc>
{
public:
    vsg::ref_ptr<vsg::RenderGraph> renderGraph;
    MEMBER(std::vector<vsg::ref_ptr<ResourceDesc>>, inputs, {});
    MEMBER(std::vector<vsg::ref_ptr<ResourceDesc>>, outputs, {});

    // In this scheme, an edge is from a resource producer to a consumer
    std::vector<vsg::ref_ptr<NodeDesc>> edges; // XXX observer_ptr?
    float resolution_scale_width  = 0.f;
    float resolution_scale_height = 0.f;

    bool compute = false;
    bool ray_tracing = false;
    bool enabled = true;
    std::string name_;
    std::string& name() { return name_; }
    NodeDesc& name(std::string n) { name_ = n; return *this; }
    vsg::ref_ptr<NodeDesc> refptr() { return vsg::ref_ptr(this); }
};
#endif

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
    // For read/modify/write resources, the output resource that provided the initial input. There will be an
    // edge between that resource's producer and the node that contains this resource as an output.
    vsg::observer_ptr<Resource> ancestor;
};

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
        : attachmentType(Output)
    {}
    AttachmentUse(const vsg::ref_ptr<Resource>& in_resource,
                  AttachmentType in_attachmentType = Output)
        : Inherit(in_resource), attachmentType(in_attachmentType),
          loadOp(VK_ATTACHMENT_LOAD_OP_DONT_CARE), storeOp(VK_ATTACHMENT_STORE_OP_DONT_CARE)
    {
    }
    AttachmentType attachmentType;
    VkAttachmentLoadOp loadOp;
    VkAttachmentStoreOp storeOp;
};


class Node : public vsg::Inherit<vsg::Object, Node>
{
public:
    
    // Are scaled dimensions needed here?
    float resolution_scale_width  = 0.f;
    float resolution_scale_height = 0.f;
    bool enabled = true;
    vsg::ref_ptr<ResourceUse> addInput(const vsg::ref_ptr<ResourceUse>& input);
    vsg::ref_ptr<ResourceUse> addWriteOnlyOutput(const vsg::ref_ptr<ResourceUse>& output);
    vsg::ref_ptr<ResourceUse> addReadWriteOutput(const vsg::ref_ptr<ResourceUse>& input,
                                                 const std::string& outputName);
    // vsg::RenderGraph
    std::vector<vsg::ref_ptr<ResourceUse>> inputs;
    std::vector<vsg::ref_ptr<ResourceUse>> outputs;
};

class RenderNode : public vsg::Inherit<Node, RenderNode>
{
public:
};

// The RenderDAG creates a graph, and then a sorted order, from the input list of nodes and resources

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
protected:
    vsg::ref_ptr<vsg::CommandGraph> _commandGraph;
    // map from resource name to resource
    std::unordered_map<std::string, vsg::ref_ptr<Resource>> resourceMap;

    std::unordered_map<std::string, vsg::ref_ptr<Node>> nodeMap;
    std::unordered_multimap<vsg::ref_ptr<Node>, vsg::ref_ptr<Node>> edges;
    void allocateTextures(vsg::Device* device);
    std::vector<vsg::ref_ptr<Node>> sortedNodes;
    void makeRenderPasses();
};
