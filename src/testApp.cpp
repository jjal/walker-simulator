#include "testApp.h"

static int pts[] = {257,219,257,258,259,274,263,325,266,345,266,352,269,369,276,387,286,415,291,425,302,451,308,462,316,472,321,480,328,488,333,495,339,501,345,505,350,507,365,515,370,519,377,522,382,525,388,527,405,534,426,538,439,539,452,539,468,540,485,540,496,541,607,541,618,539,625,537,641,530,666,513,682,500,710,476,723,463,727,457,729,453,732,450,734,447,738,440,746,423,756,404,772,363,779,343,781,339,784,327,789,301,792,278,794,267,794,257,795,250,795,232,796,222,796,197,797,195,797,188,796,188};
static int nPts  = 61*2;
static bool bReceiving=0;

//--------------------------------------------------------------
void testApp::setup() {
	ofSetVerticalSync(true);
	ofBackgroundHex(0x000000);
	ofSetLogLevel(OF_LOG_NOTICE);
	
	bMouseForce = true;
	
	box2d.init();
	box2d.setGravity(0, 10);
	box2d.createGround();
	box2d.setFPS(30.0);
	box2d.registerGrabbing();
	
	createWalker(); 

	//named pipe
	

}


void testApp::createWalker()
{
	
	ofxBox2dRevoluteJoint rHip, rKnee, rAnkle, lHip, lKnee, lAnkle;

	bodyLine.setPhysics(3.0, 0.53, 0.1);
	
	bodyLine.setup(box2d.getWorld(), 100, 200, 200, 10);

	lThigh.setPhysics(3.0, 0.53, 0.8);
	lThigh.fixture.filter.groupIndex = -8;
	lThigh.setup(box2d.getWorld(), 135,200, 100, 10);
	lHip.setup(box2d.getWorld(), bodyLine.body, lThigh.body, b2Vec2(-20.0f/OFX_BOX2D_SCALE, 0.0f/OFX_BOX2D_SCALE), b2Vec2(-100.0f/OFX_BOX2D_SCALE,0.0f/OFX_BOX2D_SCALE), 0.1f*b2_pi,.8f*b2_pi,false);
	joints.push_back(lHip);
	
	rThigh.setPhysics(3.0, 0.53, 0.8);
	rThigh.fixture.filter.groupIndex = -8;
	rThigh.setup(box2d.getWorld(), 135,200, 100, 10);
	rHip.setup(box2d.getWorld(), bodyLine.body, rThigh.body, b2Vec2(-20.0f/OFX_BOX2D_SCALE, 0.0f/OFX_BOX2D_SCALE), b2Vec2(-100.0f/OFX_BOX2D_SCALE,0.0f/OFX_BOX2D_SCALE),0.1f*b2_pi,.8f*b2_pi,false);
	joints.push_back(rHip);

	lCalf.setPhysics(3.0, 0.53, 0.8);
	lCalf.fixture.filter.groupIndex = -8;
	lCalf.setup(box2d.getWorld(), 225,200, 100, 10);
	lKnee.setup(box2d.getWorld(), lThigh.body, lCalf.body, b2Vec2(100.0f/OFX_BOX2D_SCALE, 0.0f/OFX_BOX2D_SCALE), b2Vec2(-100.0f/OFX_BOX2D_SCALE,0.0f/OFX_BOX2D_SCALE),-0.75f*b2_pi,-0.25f*b2_pi,false);
	joints.push_back(lKnee);

	rCalf.setPhysics(3.0, 0.53, 0.8);
	rCalf.fixture.filter.groupIndex = -8;
	rCalf.setup(box2d.getWorld(), 225,200, 100, 10);
	rKnee.setup(box2d.getWorld(), rThigh.body, rCalf.body, b2Vec2(100.0f/OFX_BOX2D_SCALE, 0.0f/OFX_BOX2D_SCALE), b2Vec2(-100.0f/OFX_BOX2D_SCALE,0.0f/OFX_BOX2D_SCALE),-0.75f*b2_pi,-0.25f*b2_pi,false);
	joints.push_back(rKnee);

	lFoot.setPhysics(3.0, 0.53, 0.8);
	lFoot.fixture.filter.groupIndex = -8;
	lFoot.setup(box2d.getWorld(), 315,200, 50, 10);
	lAnkle.setup(box2d.getWorld(), lCalf.body, lFoot.body, b2Vec2(100.0f/OFX_BOX2D_SCALE, 0.0f/OFX_BOX2D_SCALE), b2Vec2(-50.0f/OFX_BOX2D_SCALE,0.0f/OFX_BOX2D_SCALE),0.74f*b2_pi,0.75f*b2_pi,false);
	joints.push_back(lAnkle);

	rFoot.setPhysics(3.0, 0.53, 0.8);
	rFoot.fixture.filter.groupIndex = -8;
	rFoot.setup(box2d.getWorld(), 315,200, 50, 10);
	rAnkle.setup(box2d.getWorld(), rCalf.body, rFoot.body, b2Vec2(100.0f/OFX_BOX2D_SCALE, 0.0f/OFX_BOX2D_SCALE), b2Vec2(-50.0f/OFX_BOX2D_SCALE,0.0f/OFX_BOX2D_SCALE),0.74f*b2_pi,0.75f*b2_pi,false);
	joints.push_back(rAnkle);

	walkerPolies.push_back(bodyLine);
	walkerPolies.push_back(lThigh);
	walkerPolies.push_back(lCalf);
	walkerPolies.push_back(lFoot);
	walkerPolies.push_back(rThigh);
	walkerPolies.push_back(rCalf);
	walkerPolies.push_back(rFoot);

}

void testApp::updateWalker(vector<int> state)
{
}

vector<int> testApp::getGyroState(){
}

vector<int> testApp::readState(){
}

//--------------------------------------------------------------
void testApp::update() {
	if(bReceiving)
	{

	} else {
	box2d.update();
	for(int i=0;i<joints.size();i++)
	{	
		joints[i].setTargetAngle(1.0*cosf(time(NULL)*1000));
		joints[i].goToTargetAngle();
	}
	
	if(bMouseForce) {
		float strength = 80.0f;
		float damping  = 0.7f;
		float minDis   = 100;
		for(int i=0; i<circles.size(); i++) {
			circles[i].addAttractionPoint(mouseX, mouseY, strength);
			circles[i].setDamping(damping, damping);
		}
		bodyLine.addAttractionPoint(mouseX, mouseY, strength);
			bodyLine.setDamping(damping, damping);
		for(int i=0; i<customParticles.size(); i++) {
			customParticles[i].addAttractionPoint(mouseX, mouseY, strength);
			customParticles[i].setDamping(damping, damping);
		}
	}
	}
}

DWORD namedPipeStuff()
{
	DWORD dwError = ERROR_SUCCESS;
    PSECURITY_ATTRIBUTES pSa = NULL;
    HANDLE hNamedPipe = INVALID_HANDLE_VALUE;

    // Prepare the security attributes (the lpSecurityAttributes parameter in 
    // CreateNamedPipe) for the pipe. This is optional. If the 
    // lpSecurityAttributes parameter of CreateNamedPipe is NULL, the named 
    // pipe gets a default security descriptor and the handle cannot be 
    // inherited. The ACLs in the default security descriptor of a pipe grant 
    // full control to the LocalSystem account, (elevated) administrators, 
    // and the creator owner. They also give only read access to members of 
    // the Everyone group and the anonymous account. However, if you want to 
    // customize the security permission of the pipe, (e.g. to allow 
    // Authenticated Users to read from and write to the pipe), you need to 
    // create a SECURITY_ATTRIBUTES structure.
    if (!CreatePipeSecurity(&pSa))
    {
        dwError = GetLastError();
        wprintf(L"CreatePipeSecurity failed w/err 0x%08lx\n", dwError);
        goto Cleanup;
    }

    // Create the named pipe.
    hNamedPipe = CreateNamedPipe(
        FULL_PIPE_NAME,             // Pipe name.
        PIPE_ACCESS_DUPLEX,         // The server and client processes can 
                                    // read from and write to the pipe
        PIPE_TYPE_MESSAGE |         // Message type pipe 
        PIPE_READMODE_MESSAGE |     // Message-read mode 
        PIPE_WAIT,                  // Blocking mode is enabled
        PIPE_UNLIMITED_INSTANCES,   // Max. instances
        BUFFER_SIZE,                // Output buffer size in bytes
        BUFFER_SIZE,                // Input buffer size in bytes
        NMPWAIT_USE_DEFAULT_WAIT,   // Time-out interval
        pSa                         // Security attributes
        );

    if (hNamedPipe == INVALID_HANDLE_VALUE)
    {
        dwError = GetLastError();
        wprintf(L"Unable to create named pipe w/err 0x%08lx\n", dwError);
        goto Cleanup;
    }

    wprintf(L"The named pipe (%s) is created.\n", FULL_PIPE_NAME);
	 // Wait for the client to connect.
    wprintf(L"Waiting for the client's connection...\n");
    if (!ConnectNamedPipe(hNamedPipe, NULL))
    {
        if (ERROR_PIPE_CONNECTED != GetLastError())
        {
            dwError = GetLastError();
            wprintf(L"ConnectNamedPipe failed w/err 0x%08lx\n", dwError);
            goto Cleanup;
        }
    }
    wprintf(L"Client is connected.\n");

    // 
    // Receive a request from client.
    // 

    BOOL fFinishRead = FALSE;
    do
    {
        wchar_t chRequest[BUFFER_SIZE];
        DWORD cbRequest, cbRead;
        cbRequest = sizeof(chRequest);

        fFinishRead = ReadFile(
            hNamedPipe,     // Handle of the pipe
            chRequest,      // Buffer to receive data
            cbRequest,      // Size of buffer in bytes
            &cbRead,        // Number of bytes read
            NULL            // Not overlapped I/O
            );

        if (!fFinishRead && ERROR_MORE_DATA != GetLastError())
        {
            dwError = GetLastError();
            wprintf(L"ReadFile from pipe failed w/err 0x%08lx\n", dwError);
            goto Cleanup;
        }

        wprintf(L"Receive %ld bytes from client: \"%s\"\n", cbRead, chRequest);

    } while (!fFinishRead); // Repeat loop if ERROR_MORE_DATA

    // 
    // Send a response from server to client.
    // 

    wchar_t chResponse[] = RESPONSE_MESSAGE;
    DWORD cbResponse, cbWritten;
    cbResponse = sizeof(chResponse);

    if (!WriteFile(
        hNamedPipe,     // Handle of the pipe
        chResponse,     // Buffer to write
        cbResponse,     // Number of bytes to write 
        &cbWritten,     // Number of bytes written 
        NULL            // Not overlapped I/O
        ))
    {
        dwError = GetLastError();
        wprintf(L"WriteFile to pipe failed w/err 0x%08lx\n", dwError);
        goto Cleanup;
    }

    wprintf(L"Send %ld bytes to client: \"%s\"\n", cbWritten, chResponse);

    // Flush the pipe to allow the client to read the pipe's contents 
    // before disconnecting. Then disconnect the client's connection. 
    FlushFileBuffers(hNamedPipe);
    DisconnectNamedPipe(hNamedPipe);

Cleanup:

    // Centralized cleanup for all allocated resources.
    if (pSa != NULL)
    {
        FreePipeSecurity(pSa);
        pSa = NULL;
    }
    if (hNamedPipe != INVALID_HANDLE_VALUE)
    {
        CloseHandle(hNamedPipe);
        hNamedPipe = INVALID_HANDLE_VALUE;
    }

    return dwError;
}

//--------------------------------------------------------------
void testApp::draw() {
	
	
	for(int i=0; i<circles.size(); i++) {
		ofFill();
		ofSetHexColor(0x90d4e3);
		circles[i].draw();
	}
	
	for(int i=0; i<boxes.size(); i++) {
		ofFill();
		ofSetHexColor(0xe63b8b);
		boxes[i].draw();
	}
	
	for(int i=0; i<customParticles.size(); i++) {
		customParticles[i].draw();
	}

	
	//for(int i=0;i<walkerPolies.size();i++)
	//{
		//ofFill();
		//ofSetHexColor(0xe63b8b+i);
		//walkerPolies[i].draw();
	//}
	ofFill();
	ofSetHexColor(0xe63b8b);
	bodyLine.draw();
	ofSetHexColor(0x36bb6b);
	lThigh.draw();
	ofSetHexColor(0x46cb7b);
	rThigh.draw();
	ofSetHexColor(0x764beb);
	lCalf.draw();
	ofSetHexColor(0x865bfb);
	rCalf.draw();
	ofSetHexColor(0xb67b3b);
	lFoot.draw();
	ofSetHexColor(0xc68b4b);
	rFoot.draw();
	
	if(drawing.size()==0) polyLine.draw();
	else drawing.draw();
	
	
	string info = "";
	info += "Press [s] to draw a line strip ["+ofToString(bDrawLines)+"]\n"; 
	info += "Press [f] to toggle Mouse Force ["+ofToString(bMouseForce)+"]\n"; 
	info += "Press [c] for circles\n";
	info += "Press [b] for blocks\n";
	info += "Press [z] for custom particle\n";
	info += "Total Bodies: "+ofToString(box2d.getBodyCount())+"\n";
	info += "Total Joints: "+ofToString(box2d.getJointCount())+"\n\n";
	info += "FPS: "+ofToString(ofGetFrameRate())+"\n";
	info += "angles: "+ofToString(1.0*cosf(time(NULL)*1000))+"\n";
	ofSetHexColor(0xf0f0f0);
	ofDrawBitmapString(info, 30, 30);
	
}

//--------------------------------------------------------------
void testApp::keyPressed(int key) {
	
	if(key == 'c') {
		float r = ofRandom(4, 20);		// a random radius 4px - 20px
		ofxBox2dCircle circle;
		circle.setPhysics(3.0, 0.53, 0.1);
		circle.setup(box2d.getWorld(), mouseX, mouseY, r);
		circles.push_back(circle);
	}
	
	if(key == 'b') {
		float w = ofRandom(4, 20);	
		float h = ofRandom(4, 20);	
		ofxBox2dRect rect;
		rect.setPhysics(3.0, 0.53, 0.1);
		rect.setup(box2d.getWorld(), mouseX, mouseY, w, h);
		boxes.push_back(rect);
	}
	
	if(key == 'z') {
		float r = ofRandom(3, 10);		// a random radius 4px - 20px
		CustomParticle p;
		p.setPhysics(0.4, 0.53, 0.31);
		p.setup(box2d.getWorld(), mouseX, mouseY, r);
		p.color.r = ofRandom(20, 100);
		p.color.g = 0;
		p.color.b = ofRandom(150, 255);
		customParticles.push_back(p);
	}	
		
	if(key == 'f') bMouseForce = !bMouseForce;
	if(key == 't') ofToggleFullscreen();
	
}

//--------------------------------------------------------------
void testApp::keyReleased(int key) {
	
}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y ) {
	
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button) {
	drawing.addVertex(x, y);
}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button) {
	
	if(polyLine.isBody()) {
		drawing.clear();
		polyLine.destroy();	
	}
	
	drawing.addVertex(x, y);
}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button) {
	
	drawing.setClosed(false);
	drawing.simplify();
	
	polyLine.addVertexes(drawing);
	polyLine.simplify();
	polyLine.setPhysics(0.0, 0.5, 0.5);
	polyLine.create(box2d.getWorld());
		
	drawing.clear();

}

//--------------------------------------------------------------
void testApp::resized(int w, int h){
	
}

//   FUNCTION: CreatePipeSecurity(PSECURITY_ATTRIBUTES *)
//
//   PURPOSE: The CreatePipeSecurity function creates and initializes a new 
//   SECURITY_ATTRIBUTES structure to allow Authenticated Users read and 
//   write access to a pipe, and to allow the Administrators group full 
//   access to the pipe.
//
//   PARAMETERS:
//   * ppSa - output a pointer to a SECURITY_ATTRIBUTES structure that allows 
//     Authenticated Users read and write access to a pipe, and allows the 
//     Administrators group full access to the pipe. The structure must be 
//     freed by calling FreePipeSecurity.
//
//   RETURN VALUE: Returns TRUE if the function succeeds..
//
//   EXAMPLE CALL:
//
//     PSECURITY_ATTRIBUTES pSa = NULL;
//     if (CreatePipeSecurity(&pSa))
//     {
//         // Use the security attributes
//         // ...
//
//         FreePipeSecurity(pSa);
//     }
//
BOOL CreatePipeSecurity(PSECURITY_ATTRIBUTES *ppSa)
{
    BOOL fSucceeded = TRUE;
    DWORD dwError = ERROR_SUCCESS;

    PSECURITY_DESCRIPTOR pSd = NULL;
    PSECURITY_ATTRIBUTES pSa = NULL;

    // Define the SDDL for the security descriptor.
    PCWSTR szSDDL = L"D:"       // Discretionary ACL
        L"(A;OICI;GRGW;;;AU)"   // Allow read/write to authenticated users
        L"(A;OICI;GA;;;BA)";    // Allow full control to administrators

    if (!ConvertStringSecurityDescriptorToSecurityDescriptor(szSDDL, 
        SDDL_REVISION_1, &pSd, NULL))
    {
        fSucceeded = FALSE;
        dwError = GetLastError();
        goto Cleanup;
    }
    
    // Allocate the memory of SECURITY_ATTRIBUTES.
    pSa = (PSECURITY_ATTRIBUTES)LocalAlloc(LPTR, sizeof(*pSa));
    if (pSa == NULL)
    {
        fSucceeded = FALSE;
        dwError = GetLastError();
        goto Cleanup;
    }

    pSa->nLength = sizeof(*pSa);
    pSa->lpSecurityDescriptor = pSd;
    pSa->bInheritHandle = FALSE;

    *ppSa = pSa;

Cleanup:
    // Clean up the allocated resources if something is wrong.
    if (!fSucceeded)
    {
        if (pSd)
        {
            LocalFree(pSd);
            pSd = NULL;
        }
        if (pSa)
        {
            LocalFree(pSa);
            pSa = NULL;
        }

        SetLastError(dwError);
    }
    
    return fSucceeded;
}


//
//   FUNCTION: FreePipeSecurity(PSECURITY_ATTRIBUTES)
//
//   PURPOSE: The FreePipeSecurity function frees a SECURITY_ATTRIBUTES 
//   structure that was created by the CreatePipeSecurity function. 
//
//   PARAMETERS:
//   * pSa - pointer to a SECURITY_ATTRIBUTES structure that was created by 
//     the CreatePipeSecurity function. 
//
void FreePipeSecurity(PSECURITY_ATTRIBUTES pSa)
{
    if (pSa)
    {
        if (pSa->lpSecurityDescriptor)
        {
            LocalFree(pSa);
        }
        LocalFree(pSa);
    }
}