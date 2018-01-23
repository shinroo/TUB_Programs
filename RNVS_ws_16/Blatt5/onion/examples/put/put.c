#include <signal.h>
#include <onion/log.h>
#include <onion/onion.h>
#include <onion/shortcuts.h>

/**
 * Sanjeet
 * @short This handler just echos the content of the POST and PUT parameter 
 * of key->text
 * 
 */
onion_connection_status put_and_post_data_handler(void *_, onion_request *req, onion_response *res){
    
    
    /*read flag provided on reqiest header*/
    int flags=onion_request_get_flags(req);
    /*Extract Flags Bit and compare with onion methods*/
    int flagextraction = flags & 7; 
    
	if(flagextraction == OR_PUT){
        const char *user_data=onion_request_get_put(req,"text");
        //const onion_dict *post=  onion_request_get_post_dict(req);
        onion_response_printf(res, "The user wrote text: %s in put", user_data);
        return OCS_PROCESSED;
    }
    else if(flagextraction == OR_POST){
        const char *user_data=onion_request_get_post(req,"text");
        onion_response_printf(res, "The user wrote text: %s in post", user_data);
        return OCS_PROCESSED;
    }
    else if(flagextraction == OR_DELETE){
        const char *user_data=onion_request_get_post(req,"text");
        onion_response_printf(res, "The user wrote: %s in delete", user_data);
        return OCS_PROCESSED;
    }
    else {
        
        onion_response_printf(res, "Don't know");
        return OCS_PROCESSED;
    }
    
    
	return OCS_PROCESSED;
}

onion *o=NULL;

void onexit(int _){
	ONION_INFO("Exit");
	if (o)
		onion_listen_stop(o);
}

/**
 * This example creates a onion server and adds two urls: the base one is a static content with a form, and the
 * "data" URL is the post_data handler.
 */
int main(int argc, char **argv){
	o=onion_new(O_ONE_LOOP);
	onion_url *urls=onion_root_url(o);
    
	/***NOT REQUIRED FOR PUT : SANJEET**/
	onion_url_add_static(urls, "", 
"<html>\n"
"<head>\n"
" <title>Simple post example</title>\n"
"</head>\n"
"\n"
"Write something: \n"
"<form method=\"POST\" action=\"data\">\n"
"<input type=\"text\" name=\"text\">\n"
"<input type=\"submit\">\n"
"</form>\n"
"\n"
"</html>\n", HTTP_OK);
    /***END**/
    
	onion_url_add(urls, "data", put_and_post_data_handler);

	signal(SIGTERM, onexit);	
	signal(SIGINT, onexit);	
	onion_listen(o);

	onion_free(o);
	return 0;
}
