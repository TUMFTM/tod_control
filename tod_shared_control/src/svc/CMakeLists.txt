if (${VEHICLE})
    find_package(acados REQUIRED)
    find_package(blasfeo REQUIRED)
    find_package(hpipm REQUIRED)
    set(ACADOS_MODEL_NAME ocp_vehicle)
    set(ACADOS_SOLVER_FILES
        acados_matlab/c_generated_code/acados_solver_${ACADOS_MODEL_NAME}.c
        acados_matlab/c_generated_code/${ACADOS_MODEL_NAME}_constraints/${ACADOS_MODEL_NAME}_constr_h_fun.c
        acados_matlab/c_generated_code/${ACADOS_MODEL_NAME}_constraints/${ACADOS_MODEL_NAME}_constr_h_fun_jac_uxt_zt.c
        acados_matlab/c_generated_code/${ACADOS_MODEL_NAME}_model/${ACADOS_MODEL_NAME}_expl_ode_fun.c
        acados_matlab/c_generated_code/${ACADOS_MODEL_NAME}_model/${ACADOS_MODEL_NAME}_expl_ode_hess.c
        acados_matlab/c_generated_code/${ACADOS_MODEL_NAME}_model/${ACADOS_MODEL_NAME}_expl_vde_adj.c
        acados_matlab/c_generated_code/${ACADOS_MODEL_NAME}_model/${ACADOS_MODEL_NAME}_expl_vde_forw.c
        )
    set(TARGETS ${TARGETS} SharedVelocityController)
    add_executable(SharedVelocityController
        SharedVelocityControllerNode.cpp
        SharedVelocityController.cpp
        TrajectoryTree.cpp
        VelocityOptimizer.cpp
        ${ACADOS_SOLVER_FILES}
        ../common/ControllerBase.cpp
        ../common/CommonFeedback.cpp
        ../common/ObjectsHandler.cpp
        )
    add_dependencies(SharedVelocityController ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(SharedVelocityController ${catkin_LIBRARIES} acados blasfeo hpipm m)
    target_include_directories(SharedVelocityController PRIVATE ${catkin_INCLUDE_DIRS} acados_matlab/ ../common/)
else()
    message("skipping ${PROJECT_NAME} vehicle svc targets")
endif()

install(TARGETS ${TARGETS}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
