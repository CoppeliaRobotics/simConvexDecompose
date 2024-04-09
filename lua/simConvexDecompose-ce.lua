local codeEditorInfos = [[
int[] convexShapeHandles = simConvexDecompose.hacd(int shapeHandle, map params)
int[] convexShapeHandles = simConvexDecompose.vhacd(int shapeHandle, map params)
]]

registerCodeEditorInfos("simConvexDecompose", codeEditorInfos)
