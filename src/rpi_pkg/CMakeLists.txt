find_package(rosidl_generate_interfaces)

rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/Move.srv"
)