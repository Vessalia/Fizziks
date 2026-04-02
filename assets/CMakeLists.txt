add_library(Fizziks SHARED IMPORTED GLOBAL)

set_target_properties(Fizziks PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_LIST_DIR}/include"

    IMPORTED_LOCATION_DEBUG          "${CMAKE_CURRENT_LIST_DIR}/bin/Debug/Fizziks.dll"
    IMPORTED_LOCATION_RELEASE        "${CMAKE_CURRENT_LIST_DIR}/bin/Release/Fizziks.dll"
    IMPORTED_LOCATION_RELWITHDEBINFO "${CMAKE_CURRENT_LIST_DIR}/bin/RelWithDebInfo/Fizziks.dll"
    IMPORTED_LOCATION_MINSIZEREL     "${CMAKE_CURRENT_LIST_DIR}/bin/MinSizeRel/Fizziks.dll"

    IMPORTED_IMPLIB_DEBUG            "${CMAKE_CURRENT_LIST_DIR}/lib/Debug/Fizziks.lib"
    IMPORTED_IMPLIB_RELEASE          "${CMAKE_CURRENT_LIST_DIR}/lib/Release/Fizziks.lib"
    IMPORTED_IMPLIB_RELWITHDEBINFO   "${CMAKE_CURRENT_LIST_DIR}/lib/RelWithDebInfo/Fizziks.lib"
    IMPORTED_IMPLIB_MINSIZEREL       "${CMAKE_CURRENT_LIST_DIR}/lib/MinSizeRel/Fizziks.lib"

    IMPORTED_PDB_DEBUG               "${CMAKE_CURRENT_LIST_DIR}/bin/Debug/Fizziks.pdb"
    IMPORTED_PDB_RELWITHDEBINFO      "${CMAKE_CURRENT_LIST_DIR}/bin/RelWithDebInfo/Fizziks.pdb"
)
