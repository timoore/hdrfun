#include "RenderDAG.h"

#include <vsg/all.h>

#ifdef vsgXchange_FOUND
#    include <vsgXchange/all.h>
#endif

#include <iostream>

namespace
{
    vsg::ref_ptr<vsg::ShaderSet>
    createPbrShaderSet(const vsg::ref_ptr<vsg::Options>& options)
    {
        auto shaderSet = vsg::createPhysicsBasedRenderingShaderSet(options);
        auto vertexShader = vsg::read_cast<vsg::ShaderStage>("shaders/standard.vert", options);
        auto fragmentShader = vsg::read_cast<vsg::ShaderStage>("shaders/standard_pbr.frag", options);
        vsg::ShaderStages stages{vertexShader, fragmentShader};
        shaderSet->variants.clear();
        shaderSet->stages = stages;
        return shaderSet;
    }
}
#if 0
std::vector<vsg::ref_ptr<NodeDesc>> nodes = {
    (new NodeDesc)
    ->outputs(
        {(new ResourceDesc)
         ->resType(Attachment)
         .creationDesc((new TextureDesc)
                       ->format(VK_FORMAT_R16G16B16A16_SFLOAT)
                       .loadOp(Clear)
                       .refptr())
         .name("colorHDR")
         .refptr(),
         (new ResourceDesc)
         ->resType(Attachment)
         .creationDesc((new TextureDesc)
                       ->format(VK_FORMAT_D32_SFLOAT)
                       .loadOp(Clear)
                       .refptr())
         .name("depth")
         .refptr()})
    .name("HDR pass")
    .refptr(),
    (new NodeDesc)
    ->inputs({(new ResourceDesc)
            ->resType(Attachment)
            .name("colorHDR")
            .refptr()})
    .outputs({(new ResourceDesc)
            ->resType(Attachment)
            .name("toneMapped")
            .creationDesc((new TextureDesc)
                          ->format(VK_FORMAT_R8G8B8A8_UNORM)
                          .loadOp(DontCare)
                          .refptr()),
            (new ResourceDesc)
            ->resType(Attachment)
            .name("bloom")
            .creationDesc((new TextureDesc)
                          ->format(VK_FORMAT_R8G8B8A8_UNORM)
                          .loadOp(Clear)
                          .refptr())})
    .name("tone-mapping pass")
    .refptr(),
    (new NodeDesc)
    ->inputs({(new ResourceDesc)
            ->resType(Texture)
            .name("bloom")
            .refptr()})
    .outputs({(new ResourceDesc)
            ->resType(Attachment)
            .name("bloom-blurX")
            .creationDesc((new TextureDesc)
                          ->format(VK_FORMAT_R8G8B8A8_UNORM)
                          .loadOp(DontCare)
                          .refptr())
            .refptr()})
    .name("Bloom blur X")
    .refptr(),
    (new NodeDesc)
    ->inputs({(new ResourceDesc)
            ->resType(Attachment)
            .name("toneMapped")
            .refptr(),
            (new ResourceDesc)
            ->resType(Texture)
            .name("bloom-blurX")
            .refptr()})
    .outputs({(new ResourceDesc)
                ->resType(Attachment)
                .name("bloom-blurY / compose")
                .creationDesc((new TextureDesc)
                              ->format(VK_FORMAT_R8G8B8A8_UNORM)
                              .loadOp(DontCare)
                              .refptr())})
    
};
#endif

vsg::ref_ptr<RenderDAG>
makeHDRDag()
{
    auto dag = RenderDAG::create();
    auto HDRPassNode = RenderNode::create();
    setName(HDRPassNode, "HDRPass");
    auto HDRImage = RenderDAG::makeImage(VK_FORMAT_R16G16B16A16_UNORM);
    auto HDRIv = vsg::ImageView::create(HDRImage, VK_IMAGE_ASPECT_COLOR_BIT); // XXX delay aspect?
    auto HDRColor = ImageResource::create(HDRImage, HDRIv);
    setName(HDRColor, "HDRColor");
    dag->addResource(HDRColor);
    auto depthImage = RenderDAG::makeImage(VK_FORMAT_D32_SFLOAT);
    auto depthIv = vsg::ImageView::create(depthImage, VK_IMAGE_ASPECT_DEPTH_BIT);
    auto depth = ImageResource::create(depthImage, depthIv);
    setName(depth, "depth");
    dag->addResource(depth);
    HDRPassNode->outputs.emplace_back(AttachmentUse::create(HDRColor, Output));
    HDRPassNode->outputs.emplace_back(AttachmentUse::create(depth, Depth));
    dag->addNode(HDRPassNode);
    auto HDRResolveNode = RenderNode::create();
    HDRResolveNode->inputs.emplace_back(AttachmentUse::create(HDRColor, Input));
    HDRResolveNode->outputs.emplace_back(AttachmentUse::create(nullptr, Presentation));
    
}
    
vsg::ref_ptr<vsg::Node> createTestScene(vsg::ref_ptr<vsg::Options> options)
{
    auto builder = vsg::Builder::create();
    builder->options = options;

    auto scene = vsg::Group::create();

    vsg::GeometryInfo geomInfo;
    vsg::StateInfo stateInfo;

    scene->addChild(builder->createBox(geomInfo, stateInfo));
    geomInfo.position += geomInfo.dx * 1.5f;

    scene->addChild(builder->createSphere(geomInfo, stateInfo));
    geomInfo.position += geomInfo.dx * 1.5f;

    scene->addChild(builder->createCylinder(geomInfo, stateInfo));
    geomInfo.position += geomInfo.dx * 1.5f;

    scene->addChild(builder->createCapsule(geomInfo, stateInfo));
    geomInfo.position += geomInfo.dx * 1.5f;


    auto bounds = vsg::visit<vsg::ComputeBounds>(scene).bounds;
    double diameter = vsg::length(bounds.max - bounds.min);
    geomInfo.position.set((bounds.min.x + bounds.max.x)*0.5, (bounds.min.y + bounds.max.y)*0.5, bounds.min.z);
    geomInfo.dx.set(diameter, 0.0, 0.0);
    geomInfo.dy.set(0.0, diameter, 0.0);

    scene->addChild(builder->createQuad(geomInfo, stateInfo));

    return scene;
}

int main(int argc, char** argv)
{
    // set up defaults and read command line arguments to override them
    auto options = vsg::Options::create();
    options->paths = vsg::getEnvPaths("VSG_FILE_PATH");
    options->sharedObjects = vsg::SharedObjects::create();

#ifdef vsgXchange_all
    // add vsgXchange's support for reading and writing 3rd party file formats
    options->add(vsgXchange::all::create());
#endif
    options->setValue("linear_pipeline", true);
    options->shaderSets["pbr"] = createPbrShaderSet(options);
    
    auto windowTraits = vsg::WindowTraits::create();
    windowTraits->windowTitle = "hdrfun";


    // set up defaults and read command line arguments to override them
    vsg::CommandLine arguments(&argc, argv);
    windowTraits->debugLayer = arguments.read({"--debug", "-d"});
    windowTraits->apiDumpLayer = arguments.read({"--api", "-a"});

    arguments.read("--screen", windowTraits->screenNum);
    arguments.read("--display", windowTraits->display);
    auto numFrames = arguments.value(-1, "-f");
    if (arguments.read({"--fullscreen", "--fs"})) windowTraits->fullscreen = true;
    if (arguments.read({"--window", "-w"}, windowTraits->width, windowTraits->height)) { windowTraits->fullscreen = false; }
    if (arguments.read("--IMMEDIATE")) windowTraits->swapchainPreferences.presentMode = VK_PRESENT_MODE_IMMEDIATE_KHR;
    if (arguments.read({"-t", "--test"}))
    {
        windowTraits->swapchainPreferences.presentMode = VK_PRESENT_MODE_IMMEDIATE_KHR;
        windowTraits->fullscreen = true;
    }
    if (arguments.read("--st"))
    {
        windowTraits->swapchainPreferences.presentMode = VK_PRESENT_MODE_IMMEDIATE_KHR;
        windowTraits->width = 192, windowTraits->height = 108;
        windowTraits->decoration = false;
    }
    windowTraits->swapchainPreferences.surfaceFormat
        = {VK_FORMAT_B8G8R8A8_SRGB, VK_COLOR_SPACE_SRGB_NONLINEAR_KHR};
    // bool useStagingBuffer = arguments.read({"--staging-buffer", "-s"});

    auto outputFilename = arguments.value<std::string>("", "-o");

    bool add_ambient = true;
    bool add_directional = true;
    bool add_point = true;
    bool add_spotlight = true;
    bool add_headlight = arguments.read("--headlight");
    if (add_headlight || arguments.read({"--no-lights", "-n"}))
    {
        add_ambient = false;
        add_directional = false;
        add_point = false;
        add_spotlight = false;
    }

    vsg::ref_ptr<vsg::Node> scene;
    if (argc>1)
    {
        vsg::Path filename = argv[1];
        auto model = vsg::read_cast<vsg::Node>(filename, options);
        if (!model)
        {
            std::cout<<"Faled to load "<<filename<<std::endl;
            return 1;
        }

        scene = model;
    }
    else
    {
        scene = createTestScene(options);
    }

    // compute the bounds of the scene graph to help position camera
    auto bounds = vsg::visit<vsg::ComputeBounds>(scene).bounds;

    if (add_ambient || add_directional || add_point || add_spotlight || add_headlight)
    {
        auto span = vsg::length(bounds.max - bounds.min);
        auto group = vsg::Group::create();
        group->addChild(scene);

        // ambient light
        if (add_ambient)
        {
            auto ambientLight = vsg::AmbientLight::create();
            ambientLight->name = "ambient";
            ambientLight->color.set(1.0, 1.0, 1.0);
            ambientLight->intensity = 0.01;
            group->addChild(ambientLight);
        }

        // directional light
        if (add_directional)
        {
            auto directionalLight = vsg::DirectionalLight::create();
            directionalLight->name = "directional";
            directionalLight->color.set(1.0, 1.0, 1.0);
            directionalLight->intensity = 0.15;
            directionalLight->direction.set(0.0, -1.0, -1.0);
            group->addChild(directionalLight);
        }

        // point light
        if (add_point)
        {
            auto pointLight = vsg::PointLight::create();
            pointLight->name = "point";
            pointLight->color.set(1.0, 1.0, 0.0);
            pointLight->intensity = span*0.5;
            pointLight->position.set(bounds.min.x, bounds.min.y, bounds.max.z + span*0.3);

            // enable culling of the point light by decorating with a CullGroup
            auto cullGroup = vsg::CullGroup::create();
            cullGroup->bound.center = pointLight->position;
            cullGroup->bound.radius = span;

            cullGroup->addChild(pointLight);

            group->addChild(cullGroup);
        }

        // spot light
        if (add_spotlight)
        {
            auto spotLight = vsg::SpotLight::create();
            spotLight->name = "spot";
            spotLight->color.set(0.0, 1.0, 1.0);
            spotLight->intensity = span*0.5;
            spotLight->position.set(bounds.max.x + span*0.1, bounds.min.y - span*0.1, bounds.max.z + span*0.3);
            spotLight->direction = (bounds.min+bounds.max)*0.5 - spotLight->position;
            spotLight->innerAngle = vsg::radians(8.0);
            spotLight->outerAngle = vsg::radians(9.0);

            // enable culling of the spot light by decorating with a CullGroup
            auto cullGroup = vsg::CullGroup::create();
            cullGroup->bound.center = spotLight->position;
            cullGroup->bound.radius = span;

            cullGroup->addChild(spotLight);

            group->addChild(cullGroup);
        }

        if (add_headlight)
        {
            auto ambientLight = vsg::AmbientLight::create();
            ambientLight->name = "ambient";
            ambientLight->color.set(1.0, 1.0, 1.0);
            ambientLight->intensity = 0.1;

            auto directionalLight = vsg::DirectionalLight::create();
            directionalLight->name = "head light";
            directionalLight->color.set(1.0, 1.0, 1.0);
            directionalLight->intensity = 0.9;
            directionalLight->direction.set(0.0, 0.0, -1.0);

            auto absoluteTransform = vsg::AbsoluteTransform::create();
            absoluteTransform->addChild(ambientLight);
            absoluteTransform->addChild(directionalLight);

            group->addChild(absoluteTransform);
        }

        scene = group;
    }

    // write out scene if required
    if (!outputFilename.empty())
    {
        vsg::write(scene, outputFilename, options);
        return 0;
    }


    // create the viewer and assign window(s) to it
    auto viewer = vsg::Viewer::create();

    auto window = vsg::Window::create(windowTraits);
    if (!window)
    {
        std::cout << "Could not create windows." << std::endl;
        return 1;
    }

    viewer->addWindow(window);

    vsg::ref_ptr<vsg::LookAt> lookAt;

    vsg::dvec3 centre = (bounds.min + bounds.max) * 0.5;
    double radius = vsg::length(bounds.max - bounds.min) * 0.6;

    // set up the camera
    lookAt = vsg::LookAt::create(centre + vsg::dvec3(0.0, -radius * 3.5, 0.0), centre, vsg::dvec3(0.0, 0.0, 1.0));

    double nearFarRatio = 0.001;
    auto perspective = vsg::Perspective::create(30.0, static_cast<double>(window->extent2D().width) / static_cast<double>(window->extent2D().height), nearFarRatio * radius, radius * 10.0);

    auto camera = vsg::Camera::create(perspective, lookAt, vsg::ViewportState::create(window->extent2D()));

    // add the camera and scene graph to View
    auto view = vsg::View::create();
    view->camera = camera;
    view->addChild(scene);

    // add close handler to respond the close window button and pressing escape
    viewer->addEventHandler(vsg::CloseHandler::create(viewer));
    viewer->addEventHandler(vsg::Trackball::create(camera));

    auto renderGraph = vsg::RenderGraph::create(window, view);
    // The classic VSG background, translated into sRGB values.
    renderGraph->setClearValues({{0.033105f, 0.033105f, 0.132868f}});
    auto commandGraph = vsg::CommandGraph::create(window, renderGraph);
    viewer->assignRecordAndSubmitTaskAndPresentation({commandGraph});

    viewer->compile();

    auto startTime = vsg::clock::now();
    double numFramesCompleted = 0.0;

    // rendering main loop
    while (viewer->advanceToNextFrame() && (numFrames < 0 || (numFrames--) > 0))
    {
        // pass any events into EventHandlers assigned to the Viewer
        viewer->handleEvents();
        viewer->update();
        viewer->recordAndSubmit();
        viewer->present();

        numFramesCompleted += 1.0;
    }

    auto duration = std::chrono::duration<double, std::chrono::seconds::period>(vsg::clock::now() - startTime).count();
    if (numFramesCompleted > 0.0)
    {
        std::cout << "Average frame rate = " << (numFramesCompleted / duration) << std::endl;
    }

    return 0;
}
