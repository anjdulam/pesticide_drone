from flask import Flask, render_template, request,jsonify 
import time 
from Test import checkabort,buttonhold,buttonpitch,checksprinkler 
from dronekit import connect, VehicleMode 
pval=10
alti=10
app = Flask(__name__) 
@app.route('/') 
def index(): 
    return render_template("main.html") 
 
@app.route('/startMission') 
def startmission(): 
    return render_template("Test.html") 
 
@app.route("/", methods=["POST"]) 
def process_data(): 
    if request.method == 'POST': 
        data = request.get_json()
        time.sleep(8) 
        from Test import fly 
        fly(data,10, float(2.0), float(2.0),10) 
        return str(data) 
     
@app.route("/get_live_location") 
def get_live_location():
    connection_string = 'COM4' 
     
    vehicle = connect(connection_string, wait_ready=True,timeout=60,baud=57600) 
     
    location = vehicle.location.global_frame 
    live_location = {'lat': location.lat, 'lng': location.lon} 
    vehicle.close() 
    #live_location = {'lat':17.397228,'lng':78.490215}
    return jsonify(live_location) 
 
@app.route('/pitch',methods=['POST']) 
def abortmission(): 
    if request.method=='POST': 
        buttonpitch(True) 
        return str(1) 
     
@app.route("/stop",methods=['POST']) 
def stopping(): 
    if request.method == 'POST': 
        buttonhold(True) 
        return str(1) 
 
@app.route('/rtl',methods=['POST']) 
def triggerrtl(): 
    if request.method=='POST': 
        checkabort(True) 
        return str(1) 
 
@app.route('/sprink',methods=['POST']) 
def getsprinkler(): 
    checksprinkler(True) 
    return str(1) 
if __name__ == "__main__": 
    app.run(debug=True)

@app.route('/pestval',methods=['POST'])
def getsprinklercont():
    global pval,alti
    data=request.get_json()
    pval=data.get('number')
    alti=data.get('alti')