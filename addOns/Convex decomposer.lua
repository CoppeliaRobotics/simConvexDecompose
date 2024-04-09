#python

'''luaExec
-- We have a large Lua section within this Python script:

sim = require 'sim'

function sysCall_info()
    return {autoStart = false, menu = 'Geometry / Mesh\nConvex decomposer...'}
end

function init()
    -- First make sure the add-on ends when leaveNow is true:
    nonSimulation_orig = sysCall_nonSimulation
    function nonSimulation()
        nonSimulation_orig()
        if leaveNow then
            simUI.destroy(ui)
            ui = nil
            if params then
                local generated = {}
                if params.method < 2 then
                    local cnt = 1
                    for i = 1, #params.sel do
                        local h = params.sel[i]
                        local txt
                        if params.method == 0 then
                            txt = 'Generating HACD convex decomposed equivalent shape'
                        end
                        if params.method == 1 then
                            txt = 'Generating VHACD convex decomposed equivalent shape'
                        end
                        sim.addLog(sim.verbosity_scriptinfos, txt .. string.format(' (%i/%i)...', cnt, #params.sel)) 
                        generated[#generated + 1] = getConvexDecomposedL(h, params, params.adoptColor)
                        sim.addLog(sim.verbosity_scriptinfos, 'Done.')
                        cnt = cnt + 1
                    end
                    sim.announceSceneContentChange()
                end
                if params.method == 2 then
                    params.method = nil
                    generated = sim.callScriptFunction('generate', sim.handle_self, params)
                end
                sim.setObjectSel(generated)
                local convex = true
                for i = 1, #generated do
                    if (sim.getShapeGeomInfo(generated[i]) & 4) == 0 then
                        convex = false
                    end
                end
                if not convex then
                    sim.addLog(sim.verbosity_scripterrors, 'One or more of the generated shapes is not convex.') 
                end
            else
                if not abort then
                    simUI.msgBox(simUI.msgbox_type.info, simUI.msgbox_buttons.ok, "Convex Decomposer", 'The resulting selection is effectively empty, indicating it does not contain any non-convex shapes that meet the specified inclusion criteria..')
                    sim.setObjectSel({})
                end
            end
            return {cmd = 'cleanup'} 
        end
    end
    _G['sysCall_nonSimulation'] = nonSimulation

    simUI = require 'simUI'

    local sel = sim.getObjectSel()
    if #sel == 0 or sim.getSimulationState() ~= sim.simulation_stopped then
        simUI.msgBox(simUI.msgbox_type.info, simUI.msgbox_buttons.ok, "Convex Decomposer", 'Make sure that at least one object is selected, and that simulation is not running.')
    else
        local missingPackages = sim.callScriptFunction('checkPackages', sim.handle_self)
        if #missingPackages > 0 then
            local msg = 'You are missing some Python package that you should install with:\npip install ' .. missingPackages
            simUI.msgBox(simUI.msgbox_type.warning, simUI.msgbox_buttons.ok, "Convex Decomposer", msg)
            sim.addLog(sim.verbosity_errors, msg)
        end
        ui = simUI.create(
          [[<ui title="Convex Decomposer" closeable="true" on-close="onClose" modal="true">
            <tabs id="${ui_tab}">

                <tab title="HACD">
                <group flat="true" content-margins="0,0,0,0" layout="form">
                    <label text="min_cluster_cnt:" />
                    <spinbox id="${ui_spinMinClusterCnt}" minimum="1" maximum="100" value="1" step="1" on-change="updateUi" />
                    <label text="max_concavity:" />
                    <spinbox id="${ui_spinMaxConcavity}" minimum="0.01" maximum="100000" value="100" step="1" on-change="updateUi" />
                    <label text="max_connection_dist:" />
                    <spinbox id="${ui_spinMaxConnectionDist}" minimum="0.001" maximum="1000" value="30" step="1" on-change="updateUi" />
                    <label text="triangle_cnt_decimated_mesh:" />
                    <spinbox id="${ui_spinTriangleCntDecimatedMesh}" minimum="4" maximum="100000" value="500" step="100" on-change="updateUi" />
                    <label text="max_vertices_cnt:" />
                    <spinbox id="${ui_spinMaxVerticesCnt}" minimum="4" maximum="100000" value="200" step="100" on-change="updateUi" />
                    <label text="small_cluster_detect_threshold:" />
                    <spinbox id="${ui_spinSmallClusterDetectThreshold}" minimum="0.01" maximum="1" value="0.25" step="0.05" on-change="updateUi" />
                </group>
                <checkbox id="${ui_chAddExtraPts}" text="add_extra_pts" checked="true" on-change="updateUi" />
                <checkbox id="${ui_chAddExtraFacePts}" text="add_extra_face_pts" checked="true" on-change="updateUi" />
                </tab>
                
                <tab title="V-HACD">
                <group flat="true" content-margins="0,0,0,0" layout="form">
                    <label text="res:" />
                    <spinbox id="${ui_spinRes}" minimum="10000" maximum="64000000" value="100000" step="10000" on-change="updateUi" />
                    <label text="concavity:" />
                    <spinbox id="${ui_spinConcavity}" minimum="0.0" maximum="1.0" value="0.0025" step="0.001" on-change="updateUi" />
                    <label text="plane_downsampling:" />
                    <spinbox id="${ui_spinPlaneDownsampling}" minimum="1" maximum="16" value="4" step="1" on-change="updateUi" />
                    <label text="hull_downsampling:" />
                    <spinbox id="${ui_spinHullDownsampling}" minimum="1" maximum="16" value="4" step="1" on-change="updateUi" />
                    <label text="alpha:" />
                    <spinbox id="${ui_spinAlpha}" minimum="0.0" maximum="1.0" value="0.05" step="0.05" on-change="updateUi" />
                    <label text="beta:" />
                    <spinbox id="${ui_spinBeta}" minimum="0.0" maximum="1.0" value="0.05" step="0.05" on-change="updateUi" />
                    <label text="max_vertices:" />
                    <spinbox id="${ui_spinMaxVertices}" minimum="4" maximum="1024" value="64" step="1" on-change="updateUi" />
                    <label text="min_volume:" />
                    <spinbox id="${ui_spinMinVolume}" minimum="0.0" maximum="0.1" value="0.0001" step="0.01" on-change="updateUi" />
                </group>
                <checkbox id="${ui_chpca}" text="pca" checked="false" on-change="updateUi" />
                <checkbox id="${ui_chvoxels}" text="voxels" checked="true" on-change="updateUi" />
                </tab>
                
                <tab title="CoACD">
                <group flat="true" content-margins="0,0,0,0" layout="form">
                    <label text="threshold:" />
                    <spinbox id="${ui_spinThreshold}" minimum="0.02" maximum="1" value="0.05" step="0.01" on-change="updateUi" />
                    <label text="max_convex_hull:" />
                    <spinbox id="${ui_spinMaxConvexHull}" minimum="-1" maximum="10000" value="-1" step="10" on-change="updateUi" />
                    <label text="preprocess_mode:" />
                    <edit id="${ui_editPreprocessMode}" value="auto" on-change="updateUi" />
                    <label text="preprocess_resolution:" />
                    <spinbox id="${ui_spinPreprocessResolution}" minimum="1" maximum="1000" value="30" step="1" on-change="updateUi" />
                    <label text="resolution:" />
                    <spinbox id="${ui_spinResolution}" minimum="1" maximum="10000" value="2000" step="100" on-change="updateUi" />
                    <label text="mcts_nodes:" />
                    <spinbox id="${ui_spinMctsNodes}" minimum="1" maximum="100" value="20" step="1" on-change="updateUi" />
                    <label text="mcts_iterations:" />
                    <spinbox id="${ui_spinMctsIterations}" minimum="1" maximum="1000" value="150" step="10" on-change="updateUi" />
                    <label text="mcts_max_depth:" />
                    <spinbox id="${ui_spinMctsMaxDepth}" minimum="1" maximum="10" value="3" step="1" on-change="updateUi" />
                    <label text="seed:" />
                    <spinbox id="${ui_spinSeed}" minimum="0" maximum="10000" value="0" step="1" on-change="updateUi" />
                </group>
                <checkbox id="${ui_chPca}" text="pca" checked="false" on-change="updateUi" />
                <checkbox id="${ui_chMerge}" text="merge" checked="true" on-change="updateUi" />
                </tab>
            </tabs>
            <checkbox id="${ui_chModelShapes}" text="include model shapes" checked="false" on-change="updateUi" />
            <checkbox id="${ui_chHiddenShapes}" text="exclude hidden shapes" checked="false" on-change="updateUi" />
            <checkbox id="${ui_chAdoptColors}" text="adopt colors" checked="true" on-change="updateUi" />
            <button id="${ui_btnMorph}" text="Generate" on-click="initGenerate" />
        </ui>]]
             )
    end
end

function onClose()
    leaveNow = true
    abort = true
end

function updateUi()
end

function initGenerate()
    local includeModelShapes = simUI.getCheckboxValue(ui, ui_chModelShapes) > 0
    local excludeHiddenShapes = simUI.getCheckboxValue(ui, ui_chHiddenShapes) > 0
    local adoptColors = simUI.getCheckboxValue(ui, ui_chAdoptColors) > 0
    local s = sim.getObjectSel()
    local selMap = {}
    for i = 1, #s do
        local h = s[i]
        if sim.getModelProperty(h) == sim.modelproperty_not_model or not includeModelShapes then
            selMap[h] = true
        else
            local tree = sim.getObjectsInTree(h, sim.object_shape_type)
            for j = 1, #tree do
                selMap[tree[j]] = true
            end
        end
    end
    local sel = {}
    for obj, v in pairs(selMap) do
        if sim.getObjectType(obj) == sim.object_shape_type then
            local t = sim.getShapeGeomInfo(obj)
            if (t & 4) == 0 then
                -- not convex
                if not excludeHiddenShapes or (sim.getObjectInt32Param(obj, sim.objintparam_visible) > 0) then
                    sel[#sel + 1] = obj
                end
            end
        end
    end
    
    leaveNow = true
    if #sel > 0 then
        local method = simUI.getCurrentTab(ui, ui_tab)
        params = {adoptColor = adoptColors, sel = sel, method = method}

        if method == 2 then
            params.threshold = tonumber(simUI.getSpinboxValue(ui, ui_spinThreshold))
            params.max_convex_hull = math.floor(tonumber(simUI.getSpinboxValue(ui, ui_spinMaxConvexHull)))
            params.preprocess_mode = simUI.getEditValue(ui, ui_editPreprocessMode)
            params.preprocess_resolution = math.floor(tonumber(simUI.getSpinboxValue(ui, ui_spinPreprocessResolution)))
            params.resolution = math.floor(tonumber(simUI.getSpinboxValue(ui, ui_spinResolution)))
            params.mcts_nodes = math.floor(tonumber(simUI.getSpinboxValue(ui, ui_spinMctsNodes)))
            params.mcts_iterations = math.floor(tonumber(simUI.getSpinboxValue(ui, ui_spinMctsIterations)))
            params.mcts_max_depth = math.floor(tonumber(simUI.getSpinboxValue(ui, ui_spinMctsMaxDepth)))
            params.seed = math.floor(tonumber(simUI.getSpinboxValue(ui, ui_spinSeed)))
            params.pca = simUI.getCheckboxValue(ui, ui_chPca) > 0
            params.merge = simUI.getCheckboxValue(ui, ui_chMerge) > 0
        end
    end
end

function extractSimpleShapesL(shapes)
    local retVal = {}
    for i = 1, #shapes do
        local shape = shapes[i]
        local t = sim.getShapeGeomInfo(shape)
        if t & 1 > 0 then
            local nshapes = sim.ungroupShape(shape)
            retVal = table.add(retVal, extractSimpleShapesL(nshapes))
        else
            retVal[#retVal + 1] = shape
        end
    end
    return retVal
end

function getConvexDecomposedL(shapeHandle, params, adoptColor)
    simConvexDecompose = require('simConvexDecompose')
    local allShapes = sim.copyPasteObjects({shapeHandle})
    allShapes = extractSimpleShapesL(allShapes)
    local newShapes = {}
    for i = 1, #allShapes do
        local shape = allShapes[i]
        local parts = {}
        if params.method == 0 then
            parts = simConvexDecompose.hacd(shape, params)
        end
        if params.method == 1 then
            parts = simConvexDecompose.vhacd(shape, params)
        end
        for j = 1, #parts do
            local nshape = parts[j]
            sim.relocateShapeFrame(nshape, {0, 0, 0, 0, 0, 0, 0})
            if adoptColor then
                sim.setObjectColor(nshape, 0, sim.colorcomponent_ambient_diffuse, sim.getObjectColor(shape, 0, sim.colorcomponent_ambient_diffuse))
            end
            newShapes[#newShapes + 1] = nshape
        end
    end
    sim.removeObjects(allShapes)
    local newShape
    if #newShapes > 1 then
        newShape = sim.groupShapes(newShapes)
    else
        newShape = newShapes[1]
    end

    -- Pose, BB:
    local pose = sim.getObjectPose(shapeHandle)
    sim.relocateShapeFrame(newShape, pose)
    sim.alignShapeBB(newShape, {0, 0, 0, 0, 0, 0, 0})

    -- Dynamic aspects:
    sim.setObjectInt32Param(newShape, sim.shapeintparam_respondable, sim.getObjectInt32Param(shapeHandle, sim.shapeintparam_respondable))
    sim.setObjectInt32Param(newShape, sim.shapeintparam_respondable_mask, sim.getObjectInt32Param(shapeHandle, sim.shapeintparam_respondable_mask))
    sim.setObjectInt32Param(newShape, sim.shapeintparam_static, sim.getObjectInt32Param(shapeHandle, sim.shapeintparam_static))
    sim.setShapeMass(newShape, sim.getShapeMass(shapeHandle))
    local inertiaMatrix, com = sim.getShapeInertia(shapeHandle)
    sim.setShapeInertia(newShape, inertiaMatrix, com)
    
    -- Various:
    sim.setObjectAlias(newShape, sim.getObjectAlias(shapeHandle) .. '_convexDecomposed')
    
    return newShape
end
'''


def sysCall_init():
    sim = require('sim')
    sim.callScriptFunction('init', sim.handle_self)
    
def generate(params):
    sel = params['sel']
    del params['sel']
    adoptColor = params['adoptColor']
    del params['adoptColor']
    generated = []
    cnt = 1
    for h in sel:
        sim.addLog(sim.verbosity_scriptinfos, 'Generating CoACD convex decomposed equivalent shape ({}/{})...'.format(cnt, len(sel))) 
        generated.append(getConvexDecomposed(h, params, adoptColor))
        sim.addLog(sim.verbosity_scriptinfos, 'Done.')
        cnt += 1
    sim.announceSceneContentChange()
    return generated
    
def checkPackages():
    retVal = ''
    try:
        import numpy as np
    except:
        if len(retVal) != 0:
            retVal += ' '
        retVal += 'numpy'
    try:
        import coacd
    except:
        if len(retVal) != 0:
            retVal += ' '
        retVal += 'coacd'
    return retVal

def extractSimpleShapes(shapes):
    retVal = []
    for shape in shapes:
        t, *_ = sim.getShapeGeomInfo(shape)
        if t & 1 != 0:
            nshapes = sim.ungroupShape(shape)
            retVal.extend(extractSimpleShapes(nshapes))
        else:
            retVal.append(shape)
    return retVal

def getConvexDecomposed(shapeHandle, params, adoptColor):
    import numpy as np
    import coacd
    allShapes = sim.copyPasteObjects({shapeHandle})
    allShapes = extractSimpleShapes(allShapes)
    newShapes = []
    for shape in allShapes:
        pose = sim.getObjectPose(shape)
        vertices, indices, normals = sim.getShapeMesh(shape)
        vertices = sim.multiplyVector(pose, vertices)
        vertices = np.array(vertices).reshape(-1, 3)
        indices = np.array(indices).reshape(-1, 3)
        mesh = coacd.Mesh(vertices, indices)
        parts = coacd.run_coacd(mesh, **params)
        for part in parts:
            vert = part[0].flatten().tolist()
            ind = part[1].flatten().tolist()
            nshape = sim.createShape(0, 0.0, vert, ind)
            sim.relocateShapeFrame(nshape, [0, 0, 0, 0, 0, 0, 0])
            if adoptColor:
                sim.setObjectColor(nshape, 0, sim.colorcomponent_ambient_diffuse, sim.getObjectColor(shape, 0, sim.colorcomponent_ambient_diffuse))
            newShapes.append(nshape)
    sim.removeObjects(allShapes)
    if len(newShapes) > 1:
        newShape = sim.groupShapes(newShapes)
    else:
        newShape = newShapes[0]

    # Pose, BB:
    pose = sim.getObjectPose(shapeHandle)
    sim.relocateShapeFrame(newShape, pose)
    sim.alignShapeBB(newShape, [0, 0, 0, 0, 0, 0, 0])

    # Dynamic aspects:
    sim.setObjectInt32Param(newShape, sim.shapeintparam_respondable, sim.getObjectInt32Param(shapeHandle, sim.shapeintparam_respondable))
    sim.setObjectInt32Param(newShape, sim.shapeintparam_respondable_mask, sim.getObjectInt32Param(shapeHandle, sim.shapeintparam_respondable_mask))
    sim.setObjectInt32Param(newShape, sim.shapeintparam_static, sim.getObjectInt32Param(shapeHandle, sim.shapeintparam_static))
    sim.setShapeMass(newShape, sim.getShapeMass(shapeHandle))
    inertiaMatrix, com = sim.getShapeInertia(shapeHandle)
    sim.setShapeInertia(newShape, inertiaMatrix, com)
    
    # Various:
    sim.setObjectAlias(newShape, sim.getObjectAlias(shapeHandle) + '_convexDecomposed')
    
    return newShape