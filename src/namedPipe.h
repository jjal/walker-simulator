#pragma region Includes
#include <stdio.h>
#include <windows.h>
#include <sddl.h>
#pragma endregion


// The full name of the pipe in the format of \\servername\pipe\pipename.
#define SERVER_NAME         L"."
#define PIPE_NAME           L"robotpipe"
#define FULL_PIPE_NAME      L"\\\\" SERVER_NAME L"\\pipe\\" PIPE_NAME

#define BUFFER_SIZE     1024

// Response message from client to server. '\0' is appended in the end 
// because the client may be a native C++ application that expects NULL 
// termiated string.
#define RESPONSE_MESSAGE    L"Default response from server"


BOOL CreatePipeSecurity(PSECURITY_ATTRIBUTES *);
void FreePipeSecurity(PSECURITY_ATTRIBUTES);
