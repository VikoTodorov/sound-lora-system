from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

from chirpstack_api.as_pb import integration
from google.protobuf.json_format import Parse

from dbWrite import dbWrite

class Handler(BaseHTTPRequestHandler):
    # True -  JSON marshaler
    # False - Protobuf marshaler (binary)
    json = True

    def do_POST(self):
        query_args = parse_qs(urlparse(self.path).query)

        content_len = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(content_len)

        if query_args["event"][0] == "up":
            self.up(body)

        elif query_args["event"][0] == "join":
            self.send_response(200, "OK!")
            self.end_headers()

        else:
            self.send_response(404, "NOT FOUND!")
            self.end_headers()

    def up(self, body):
        uplinkData = self.unmarshal(body, integration.UplinkEvent())
        writeToDBFlag = dbWrite(application_name, uplinkData.dev_eui.hex(),
                                uplinkData.object_json)

        if writeToDBFlag:
            self.send_response(200, "OK!")
            self.end_headers()
        else:
            self.send_response(418, "Your data is incorrect!")
            self.end_headers()

    def unmarshal(self, body, pl):
        if self.json:
            return Parse(body, pl)

        pl.ParseFromString(body)
        return pl

httpd = HTTPServer(('', 8090), Handler)
httpd.serve_forever()
