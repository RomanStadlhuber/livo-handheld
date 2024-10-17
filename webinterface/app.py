from flask import Flask, render_template, redirect, url_for, request
from flask_wtf import FlaskForm
from wtforms import SelectField, StringField, SubmitField, FileField
from wtforms.validators import DataRequired, Optional, Regexp
from flask_bootstrap import Bootstrap5
from interfaces import Interfaces
from state import State
import re

app = Flask(__name__)
app.secret_key = "lic"
# set dark theme, see
# https://bootstrap-flask.readthedocs.io/en/stable/advanced/#bootswatch-themes
# and
# https://bootswatch.com/
app.config["BOOTSTRAP_BOOTSWATCH_THEME"] = "cyborg"
bootstrap = Bootstrap5(app)

# instance to attach to interfaces
interfaces = Interfaces()
# recorder state management
state = State()

class StartRecordingForm(FlaskForm):
    storage_location = SelectField(
        "Storage Location",
        validators=[DataRequired()],
        choices=Interfaces.get_storage_devices(),
        description="Select a storage location to save recordings to.",
    )
    recording_name = StringField(
        label="Name of Recording (optional).",
        description="Note that the '.bag' extension will be applied automatically.",
        validators=[
            Optional(),
            Regexp(re.compile("^[\w\-. ]+$", re.IGNORECASE), message="Please use a valid filename."),
        ],
        render_kw=dict(placeholder="prefix of the generated rosbag"),
    )
    submit = SubmitField(label="Start Recording")


class StopRecordingForm(FlaskForm):
    submit = SubmitField(label="Stop Recording")



@app.route("/index")
@app.route("/", methods=["GET", "POST"])
def index():
    # if there's currently an active recording, redirect to rec. page
    if state.currently_recording:
        return redirect(
            url_for(
                "record",
                basepath=state.recording_basepath,
                filename=state.recording_filename,
            )
        )
    form = StartRecordingForm()
    if form.validate_on_submit():
        return redirect(
            url_for(
                "record",
                basepath=request.form.get("storage_location"),
                filename=request.form.get("recording_name"),
            )
        )
    # pick selected or first (= default) storage location
    rosbag_storage_location = request.form.get("storage_location") or Interfaces.get_storage_devices()[0]
    # get list of recordings in that storage location
    bag_list = interfaces.get_bags(rosbag_storage_location)
    return render_template("index.html", form=form, interfaces=interfaces, rosbags=bag_list)


@app.route("/record", methods=["GET", "POST"])
def record():
    form = StopRecordingForm()
    if form.validate_on_submit():
        interfaces.stop_recording()
        # TODO: do we really need to stop device nodes?
        interfaces.stop_device_nodes()
        state.set_recording_stopped()
        return redirect("/")
    # resume active recording
    if state.currently_recording:
        status = dict(
            camera="Running" if state.cam_imu_running else "Unavailable",
            lidar="Running" if state.lidar_running else "Unavailable",
            basepath=state.recording_basepath,
            filename=state.recording_filename
        )
        print("::: resuming currently active recording :::")
        print(status)
        return render_template("recording.html", form=form, status=status)
    # start new recording
    else:
        basepath = request.args.get("basepath")
        filename = request.args.get("filename")
        devices = interfaces.start_device_nodes()
        bagname = interfaces.start_recording(basepath, filename)
        state.set_recording_path(basepath, bagname)
        state.set_devices_running(
                cam_imu=devices["camera"],
                lidar=devices["lidar"],
            )
        status = dict(
            camera="Running" if devices["camera"] else "Unavailable",
            lidar="Running" if devices["lidar"] else "Unavailable",
            basepath=basepath,
            filename=bagname,
        )
        print(status)
        return render_template("recording.html", form=form, status=status)


if __name__ == "__main__":
    app.run(host='0.0.0.0') # app.run(debug=True)
