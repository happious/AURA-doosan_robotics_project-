from app import create_app


app = create_app()


if __name__ == "__main__":
    print(
        "[AURA UI] startup "
        f"robot={app.config['ROBOT_ID']} "
        f"node={app.config['ROS_NODE_NAME']} "
        f"port={app.config['PORT']} "
        f"control_mode={app.config['CONTROL_MODE']} "
        f"alignment_mode={app.config['ALIGNMENT_INPUT_MODE']}",
        flush=True,
    )

    app.run(
        host=app.config["HOST"],
        port=app.config["PORT"],
        debug=app.config["DEBUG"],
        use_reloader=False,
        threaded=True,
    )
