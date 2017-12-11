#ifndef _HTTP_RESPONSE_MULTIPART_PARSER_H_
#define _HTTP_RESPONSE_MULTIPART_PARSER_H_

#include "multipart_parser.h"
#include "http_response.h"

class HttpResponseMultipartParser {
public:
    HttpResponseMultipartParser(HttpResponse* a_response, const char* boundary, Callback<void(const char *at, size_t length)> a_body_callback = 0)
        : response(a_response), body_callback(a_body_callback)
	{
		memset(&m_callbacks, 0, sizeof(multipart_parser_settings));
        m_callbacks.on_header_field = read_header_name;
        m_callbacks.on_header_value = read_header_value;

        m_parser = multipart_parser_init(boundary, &m_callbacks);
        multipart_parser_set_data(m_parser, (void*)this);
	}

	~HttpResponseMultipartParser()
    {
        multipart_parser_free(m_parser);
    }

    size_t execute(const char* body, size_t len)
    {
        multipart_parser_execute(m_parser, body, len);
        return m_headers;
    }

private:
	static int read_header_name(multipart_parser* p, const char *at, size_t length)
    {
        HttpResponseMultipartParser* me = (HttpResponseMultipartParser*)multipart_parser_get_data(p);
        me->m_headers++;
    }
    static int read_header_value(multipart_parser* p, const char *at, size_t length)
    {
       printf("%.*s\n", length, at);
       return 0;
    }

    HttpResponse* response;
    Callback<void(const char *at, size_t length)> body_callback;
    multipart_parser* m_parser;
    multipart_parser_settings m_callbacks;
    size_t m_headers;
};
#endif