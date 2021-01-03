#include "telemetry.h"

#include <cstring>
#include <esp_log.h>

const char* LOG = "telemetry";

Telemetry::Telemetry() {
    initConnection();
}

void Telemetry::initConnection() {
    esp_http_client_config_t config {
        .url = "http://10.13.35.196:8080/telemetry",
        .host = NULL, // overridden by url
        .port = 0, // overridden by url
        .username = NULL,
        .password = NULL,
        .auth_type = HTTP_AUTH_TYPE_NONE,
        .path = NULL, // overridden by url
        .query = NULL, // overridden by url
        .cert_pem = NULL,
        .client_cert_pem = NULL,
        .client_key_pem = NULL,
        .user_agent = "esp32",
        .method = HTTP_METHOD_POST,
        .timeout_ms = 1000,
        .disable_auto_redirect = false,
        .max_redirection_count = 0,
        .max_authorization_retries = -1,
        .event_handler = httpEventHandler,
        .transport_type = HTTP_TRANSPORT_OVER_TCP,
        .buffer_size = 32,
        .buffer_size_tx = 1024,
        .user_data = NULL,
        .is_async = false,
        .use_global_ca_store = false,
        .skip_cert_common_name_check = false
    };
    httpClient = esp_http_client_init(&config);
}

esp_err_t Telemetry::httpEventHandler(esp_http_client_event_t* event) {
    switch(event->event_id) {
    case HTTP_EVENT_ERROR:
        ESP_LOGI(LOG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        break;
    case HTTP_EVENT_HEADER_SENT:
        break;
    case HTTP_EVENT_ON_HEADER:
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGI(LOG, "HTTP_EVENT_ON_DATA, len=%d", event->data_len);
        if (!esp_http_client_is_chunked_response(event->client)) {
            printf("%.*s", event->data_len, (char*)event->data);
        }
        break;
    case HTTP_EVENT_ON_FINISH:
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGW(LOG, "HTTP_EVENT_DISCONNECTED");
        break;
    }

    return ESP_OK;
}

void Telemetry::postTelemetry(const char* data) {
    esp_http_client_set_post_field(httpClient, data, strlen(data));
    esp_err_t error = esp_http_client_perform(httpClient);

    if (error != ESP_OK) {
        // retry once
        initConnection();
        esp_http_client_set_post_field(httpClient, data, strlen(data));
    }
}
