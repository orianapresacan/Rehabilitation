from flask import Flask, render_template
import matplotlib.pyplot as plt
from io import BytesIO
import base64

app = Flask(__name__)

@app.route('/')
def home():
    return render_template('home.html')

@app.route('/patients')
def patients():
    # test array (database)
    rawData = [
        "brush teeth:120",
        "eating:200",
        "brush teeth:90",
    ]

    activityDurations = {}
    for item in rawData:
        activity, durationStr = item.split(':')
        duration = int(durationStr) / 60  # convert seconds to minutes
        activityDurations[activity] = activityDurations.get(activity, 0) + duration

    fig, ax = plt.subplots()
    activities = list(activityDurations.keys())
    durations = [activityDurations[activity] for activity in activities]
    ax.bar(activities, durations)
    ax.set_ylabel('Duration (minutes)')

    buf = BytesIO()
    plt.savefig(buf, format="png")
    buf.seek(0)
    image_url = base64.b64encode(buf.read()).decode('utf-8')

    # Modify the image_url assignment
    image_url = f"data:image/png;base64,{image_url}"

    return render_template('patients.html', image_url=image_url)

if __name__ == '__main__':
    app.run(debug=True)
