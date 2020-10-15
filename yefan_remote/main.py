from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
from yefan_api import robot
import rospy
import signal
import time
from yefan_api import fk, ik

PORT = 8080


def get_ip():
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    ip = s.getsockname()[0]
    s.close()
    return ip


def file_to_str(filename):
    fin = open(filename)
    contents = fin.read()
    fin.close()
    return contents


cmd_pubs = {}
joints = []


def prepare_page():
    sliders = []
    slider_tmpl = file_to_str('slider_template.html')
    joint_states = robot.get_joint_states()
    server_url = "http://" + get_ip() + ":" + str(PORT)
    for joint in joints:
        joint_name = joint.joint
        min_value = joint.min_pos
        max_value = joint.max_pos
        curr = joint_states.position(joint_name)

        sliders.append(slider_tmpl.format(**locals()))

    body = "".join(sliders)
    server_url = get_ip() + ":" + str(PORT)
    return file_to_str('page_template.html').format(**locals())


def get_curr_pos():
    joint_states = robot.get_joint_states()
    s = ""
    js = []
    for j in joints:
        p = joint_states.position(j.joint)
        s += j.joint + ": " + str(p) + "\n"
        js.append(p)
    h = fk.calculate(js)
    s += "x: " + str(h[0][3]) + "\n"
    s += "y: " + str(h[1][3]) + "\n"
    s += "z: " + str(h[2][3]) + "\n"
    s += "\nfull H: " + str(h) + "\n\n"

    ii = ik.calculate(h)

    s += "\nIK: " + str(ii) + "\n\n"
    return s


class RequestHandler(BaseHTTPRequestHandler):
    def _set_headers(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()

    def do_GET(self):
        self._set_headers()
        s = prepare_page()
        self.wfile.write(s)

    def do_HEAD(self):
        self._set_headers()

    def do_POST(self):
        self._set_headers()
        data_string = self.rfile.read(int(self.headers['Content-Length']))
        joint_name, value_str = data_string.split("=")
        cmd_pubs[joint_name].publish(float(value_str))
        self.wfile.write(get_curr_pos())


def run(server_class=HTTPServer, handler_class=RequestHandler, port=80):
    server_address = ('', port)
    httpd = server_class(server_address, handler_class)

    prepare_shutdown(httpd)

    print 'Starting httpd...'
    httpd.serve_forever()


def prepare_shutdown(server):
    def shutdown_server():
        print('Server shutdowning! Wait please...(max is 60 seconds)')
        start_time = time.time()
        server.shutdown()
        print('Done! In ' + str(time.time() - start_time) + ' seconds')

    def shutdown_handler(signal, frame):
        print('You pressed Ctrl+C!')
        import threading
        t = threading.Thread(target=shutdown_server)
        t.start()

    signal.signal(signal.SIGINT, shutdown_handler)


def main():
    rospy.init_node('remote_control', anonymous=True)

    global joints
    joints = robot.all_joints
    for joint in joints:
        cmd_pubs[joint.joint] = joint.command_publisher()

    run(port=PORT)


main()
