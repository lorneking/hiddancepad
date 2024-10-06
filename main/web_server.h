#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include "esp_http_server.h"
#include "esp_log.h"

// HTTP Server Handlers

esp_err_t hello_get_handler(httpd_req_t *req);
esp_err_t hx711_get_handler(httpd_req_t *req);
void start_webserver(void);

#endif // WEB_SERVER_H