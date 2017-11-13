#include "ofxTriangleMesh.h"
#include "triangle.h"


void triangulatePoints(char * flags, triangulateio * in, triangulateio * mid, 
                       triangulateio * out){

    // this funciton, which calls triangulage is because we have a function called triangulate, so the compiler get's a bit confused.
    triangulate(flags, in,  mid, out);
}


ofxTriangleMesh::ofxTriangleMesh(){
    nTriangles = 0;
}


// see note in the h file for how to use the parameters here....
void ofxTriangleMesh::triangulate(ofPolyline contour, ofPolyline hole, float angleConstraint, float sizeConstraint){

    int numContourPoints = contour.size();
    int numHolePoints    = hole.size();
    int numPoints = numContourPoints + numHolePoints;
    int numHoles = 1; // Number of hole regions.
   
    struct triangulateio in, out;
    in.numberofpoints = numPoints;
    in.numberofpointattributes = 0;
    in.pointlist = (REAL *)malloc(numPoints * 2 * sizeof(REAL));
    in.pointmarkerlist = (int *)  malloc(numPoints * 1 * sizeof(int));
    in.numberofregions = 0;
    in.regionlist =  NULL;
    
    // Number of line segments.
    in.numberofsegments = numPoints;
    in.segmentlist       = (int *)malloc(in.numberofsegments * 2 * sizeof(int));
    in.segmentmarkerlist = (int *)malloc(in.numberofsegments * 1 * sizeof(int));

    in.numberofholes = 1;//hole.size();
    in.holelist = (REAL *)malloc(in.numberofholes * 2 * sizeof(REAL));
    

    // Populate point list.   
    for(int i = 0; i < numContourPoints; i++){
		in.pointlist[i*2+0] = contour[i].x;
        in.pointlist[i*2+1] = contour[i].y;
        in.pointmarkerlist[i] = 1;// 1 means contour number 1, in this case the outer one.
    }
    for(int i = 0; i < numHolePoints; i++)
    {
        int index = i + numContourPoints;
        in.pointlist[index*2 + 0] = hole[i].x;
        in.pointlist[index*2 + 1] = hole[i].y;
        in.pointmarkerlist[index] = 2; // 2 means contour number 2, in this case the hole.
    }

    // Populate the segment list.
    for (int i = 0; i < numContourPoints; i++) {
        in.segmentlist[i*2 + 0] = i;
        in.segmentlist[i*2 + 1] = (i + 1) % numContourPoints;
        in.segmentmarkerlist[i] = 1;// 1 means contour number 1, in this case the outer one.
    }
    for (int i = 0; i < numHolePoints; i++)
    {
        int index = i + numContourPoints;
        in.segmentlist[index * 2 + 0] = index;
        in.segmentlist[index * 2 + 1] = ((i + 1) % numHolePoints) + numContourPoints;
        in.segmentmarkerlist[index] = 2; // 2 means contour number 2, in this case the hole.
    }

    // Populate hole point list.
    ofPoint center = hole.getCentroid2D();
    in.holelist[0 + 0] = center.x;// x of infection.
    in.holelist[0 + 1] = center.y;// y of infection.

    out.pointlist = (REAL *) NULL;            
    out.pointattributelist = (REAL *) NULL;
    out.pointmarkerlist = (int *) NULL; 
    out.trianglelist = (int *) NULL;  
    out.triangleattributelist = (REAL *) NULL;
    out.neighborlist = (int *) NULL; 
    out.segmentlist = (int *) NULL;
    out.segmentmarkerlist = (int *) NULL;
    out.edgelist = (int *) NULL;
    out.edgemarkerlist = (int *) NULL; 
    out.holelist = (REAL *) NULL; 

    
    bool bConstrainAngle = false;
    bool bConstrainSize = false;
    
    if (angleConstraint > 0) bConstrainAngle = true;
    if (sizeConstraint > 0) bConstrainSize = true;
    
    
    
    string triangulateParams;
    
    triangulateParams += "z";   // start from zero
    triangulateParams += "Y";   // Prohibits the insertion of Steiner points (extra points) on the mesh boundary.
    triangulateParams += "V";//"Q";   // quiet!   change to V is you want alot of info
    
    if (bConstrainAngle == true){
        triangulateParams += "q" + ofToString( angleConstraint );
    }
    
    if (bConstrainSize == true){
        triangulateParams += "a" + ofToString( (int)sizeConstraint );
    }

    triangulateParams += "p"; // Assumes input is a .poly planar line graph file, so it will carve holes.
    
    
    triangulatePoints((char *) triangulateParams.c_str(), &in, &out, NULL);



    /*
    printf("Initial triangulation:\n\n");
    //report(&mid, 1, 1, 1, 1, 1, 0);
        for (int i = 0; i < out.numberofpoints; i++) {
            printf("Point %4d:", i);
            for (int j = 0; j < 2; j++) {
                printf("  %.6g", out.pointlist[i * 2 + j]);
            }
            printf("\n");

        }
    */
    
    
    nTriangles = 0;
    triangles.clear();
    
    
    std::map < int , ofPoint  > goodPts;
    
    for (int i = 0; i < out.numberoftriangles; i++) {
        meshTriangle triangle;
        
        int whichPt;
        
        for (int j = 0; j < 3; j++){
            whichPt = out.trianglelist[i * 3 + j];
            triangle.pts[j] = ofPoint(  out.pointlist[ whichPt * 2 + 0],  out.pointlist[ whichPt * 2 + 1]);
            triangle.index[j] = whichPt;
            
            
            
        }
        
        ofPoint tr[3];
        tr[0] = triangle.pts[0];
        tr[1] = triangle.pts[1];
        tr[2] = triangle.pts[2];
		
        
        // here we check if a triangle is "inside" a contour to drop non inner triangles
        
        if( isPointInsidePolygon(&contour[0], contour.size(), getTriangleCenter(tr) ) ) {
            triangle.randomColor = ofColor(ofRandom(0,255), ofRandom(0,255), ofRandom(0,255));
            triangles.push_back(triangle);
            
            // store the good points in a map
            for (int j = 0; j < 3; j++){
                goodPts[triangle.index[j]] = triangle.pts[j]; 
            }
            nTriangles++;
        }
    }
    
    // put all good points in a vector and handle the remapping of indices.
    // the indices stored above were for all points, but since we drop triangles, we 
    // can drop non used points, and then remap all the indces. 
    // that happens here: 
    
    outputPts.clear();
    std::map < int, int > indexChanges;
    std::map< int , ofPoint >::iterator iter;
    for (iter = goodPts.begin(); iter != goodPts.end(); ++iter) {
        //cout << iter->first << " " << iter->second << endl;
        indexChanges[iter->first] = outputPts.size();
        outputPts.push_back(iter->second);
    }
    
    // now, with the new, potentially smaller group of points, update all the indices of the triangles so their indices point right: 
    
    for (int i = 0; i < triangles.size(); i++){
        for (int j = 0; j < 3; j++){
            triangles[i].index[j] = indexChanges[triangles[i].index[j]];
        }
    }
    
    // now make a mesh, using indices: 
    
    triangulatedMesh.clear();
    triangulatedMesh.setMode(OF_PRIMITIVE_TRIANGLES);
    for (int i = 0; i < outputPts.size(); i++){
        triangulatedMesh.addVertex(outputPts[i]);
    }
    
    for (int i = 0; i < triangles.size(); i++){
        triangulatedMesh.addIndex(triangles[i].index[0]);;
        triangulatedMesh.addIndex(triangles[i].index[1]);;
        triangulatedMesh.addIndex(triangles[i].index[2]);;
    }

    // depending on flags, we may need to adjust some of the memory clearing
    // (see tricall.c for full listings)
    // TODO: this should be agressively tested. 
    
    free(in.pointlist);
    free(out.pointlist);
    if (out.pointattributelist != NULL) free(out.pointattributelist);
    if (out.pointmarkerlist != NULL) free(out.pointmarkerlist);
    free(out.trianglelist);
    if (out.triangleattributelist != NULL) free(out.triangleattributelist);
    
    return;

}

void ofxTriangleMesh::clear(){
    triangles.clear();
    nTriangles = 0;
}

ofPoint ofxTriangleMesh::getTriangleCenter(ofPoint *tr){
    float c_x = (tr[0].x + tr[1].x + tr[2].x) / 3;
    float c_y = (tr[0].y + tr[1].y + tr[2].y) / 3;
    return ofPoint(c_x, c_y);
}

bool ofxTriangleMesh::isPointInsidePolygon(ofPoint *polygon,int N, ofPoint p)
{
    int counter = 0;
    int i;
    double xinters;
    ofPoint p1,p2;

    p1 = polygon[0];

    for (i=1;i<=N;i++)
    {
        p2 = polygon[i % N];
        if (p.y > MIN(p1.y,p2.y)) {
            if (p.y <= MAX(p1.y,p2.y)) {
                if (p.x <= MAX(p1.x,p2.x)) {
                    if (p1.y != p2.y) {
                        xinters = (p.y-p1.y)*(p2.x-p1.x)/(p2.y-p1.y)+p1.x;
                        if (p1.x == p2.x || p.x <= xinters){
                            counter++;
						}
                    }
                }
            }
        }
        p1 = p2;
    }
	return counter % 2 != 0;
}


void ofxTriangleMesh::draw() {

    // draw the triangles in their random colors: 
    
    for (int i=0; i<nTriangles; i++){
    
        ofFill();
        ofSetColor( triangles[i].randomColor);
        ofTriangle( outputPts[triangles[i].index[0]], 
        outputPts[triangles[i].index[1]],
                   outputPts[triangles[i].index[2]]);
        
    }
    
    // draw the mesh as a wire frame in white on top. 
    
    ofSetColor(255,255,255);
    triangulatedMesh.drawWireframe();
    
}
