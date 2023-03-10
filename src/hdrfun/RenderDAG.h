#pragma once

// A VSG take on the render graph implementation from Mastering Graphics

#include <vsg/core/Inherit.h>
#include <vsg/state/ImageView.h>
#include <vsg/state/Sampler.h>

#include <limits>

enum FrameGraphResourceType
{
  Invalid = -1,
  Buffer = 0,
  Texture = 1,
  Attachment = 2,
  Reference = 3
};

// An operation, which can be either load or store
enum RenderPassOperation
{
    DontCare,
    Load,
    Clear
};

class FrameGraphNode;

template <typename T>
uint32_t backIndex(const T& vector)
{
    return static_cast<uint32_t>(&vector.back() - vector.data());
}

// The Desc classes are holders of data about render passes and resources. They
// are relatively easy to initalize by hand.

// Macro arguments can't contain literal commas!
#define COMMA ,
#define MEMBER(TYPE, NAME, INIT) TYPE& NAME() { return NAME ## _; }     \
    auto NAME(const TYPE& val) { NAME ## _ = val; return *this; } \
    TYPE NAME ## _ = INIT;

class CreationDesc : public vsg::Inherit<vsg::Object, CreationDesc>
{
public:
    MEMBER(bool, external, false);
    MEMBER(bool, compute, false)
};

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

class ResourceDesc : public vsg::Inherit<vsg::Object, ResourceDesc>
{
    public:
    MEMBER(vsg::ref_ptr<CreationDesc>, creationDesc, {});
    MEMBER(FrameGraphResourceType, resType, Invalid);

    vsg::observer_ptr<FrameGraphNode> producer;
    int refCount;               // needed?
    std::string name_;
    std::string& name() { return name_; }
    ResourceDesc& name(const std::string& n) { name_ = n; return *this; }
    vsg::ref_ptr<TextureDesc> refptr() { return vsg::ref_ptr(this); }
};

// Our graph node will contain a RenderGraph, which encapsulates a frame buffer,
// its render pass, and a view which contains the scene graph to render.
//
// Why not call it a PassDesc? It could be something other than a RenderPass e.g., compute.

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

// These classes are the nodes and edges of the DAG. The contain the
// description, bookkeeping for sorting the graph, and eventually VSG objects
// that will instantiate the CommandGraph.
//
// These are stored in std:vector objects. References to them are by index.

typedef uint32_t NodeHandle;
typedef uint32_t ResourceHandle;

const uint32_t invalidHandle = std::numeric_limits<uint32_t>::max();

struct Resource
{
    explicit Resource(const vsg::ref_ptr<ResourceDesc>& resDesc)
        : desc(resDesc), producer(invalidHandle)
    {
        resType = desc->resType();
    }
    
    // This is the input type. Redundant with type in desc?
    FrameGraphResourceType resType;
    vsg::ref_ptr<ResourceDesc> desc;
    NodeHandle producer;        // Node that produces this resource as output
    ResourceHandle outputHandle; // Source output resource
};

struct Node
{
    explicit Node(const vsg::ref_ptr<NodeDesc>& nodeDesc)
        : desc(nodeDesc)
    {
    }
    vsg::ref_ptr<NodeDesc> desc;
    // vsg::RenderGraph
    std::vector<ResourceHandle> inputs;
    std::vector<ResourceHandle> outputs;
};

// The RenderDAG creates a graph, and then a sorted order, from the input list of nodes and resources

class RenderDAG
{
public:
    RenderDAG(std::vector<vsg::ref_ptr<NodeDesc>>& nodes);
    // Dimensions that are the basis for the scaling parameters of resources. Need to handle
    // updating all those after a resize
    VkExtent2D dimensions;
    Resource& addResource(const vsg::ref_ptr<ResourceDesc>& desc, bool isOutput);
    void computeEdges();
    void build(vsg::Device* device);
    vsg::CommandGraph getCommandGraph();
    vsg::RenderGraph getRenderGraph(const std::string& name);
protected:
    vsg::ref_ptr<vsg::CommandGraph> _commandGraph;
    // Vector of all references to resources in inputs and outputs
    std::vector<Resource> resources;
    // map from resource name to resource reference that creates it i.e., the output that creates it.
    std::unordered_map<std::string, ResourceHandle> resourceMap;
    std::vector<Node> nodes;

    std::unordered_map<std::string, NodeHandle> nodeMap;
    std::unordered_multimap<NodeHandle, NodeHandle> edges;
    void allocateTextures(vsg::Device* device);
    void setInputUsage(ResourceHandle resHandle);
    void setOutputUsage(ResourceHandle resHandle);
    std::vector<NodeHandle> sortedNodes;
    void makeRenderPasses();
};
