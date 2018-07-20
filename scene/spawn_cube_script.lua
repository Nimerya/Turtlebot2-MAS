--[[
    The following function was added in the main script of the scene: 
    it create the package that will be loaded on one of the robots.
--]]

spawnCube=function(inInts,inFloats,inStrings,inBuffer)
	-- inInts, inFloats and inStrings are tables
	-- inBuffer is a string
    
    -- Perform any type of operation here.
    local handle = sim.createPureShape(0,8,{0.15,0.15,0.15},3,nil)

	-- Always return 3 tables and a string, e.g.:
	return {handle},{},{},''
end
