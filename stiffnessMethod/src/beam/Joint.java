/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package beam;



/**
 *
 * @author Kamran
 */
public class Joint {
    //Coordinate
    private double x;
    private double y;
    
    //Following two variables are used for joint Nobs    
    private int xNo;
    private int yNo;
    
    //following two variables are used for support conditions.
    //Since only two directions are involved in truss, 
    //therefore we took x & y    
    private boolean xRestrained=false;
    private boolean yRestrained=false;
    
    //forces on joint
    private double fx;
    private double fy;
    boolean isFxSet;
    boolean isFySet;
    
    //displacement on joint
    private double dx;
    private double dy;

    public double getFx() {
        return fx;
    }
    public void setFx(double fx) {
        this.fx = fx;
    }

    public double getFy() {
        return fy;
    }

    public void setFy(double fy) {
        this.fy = fy;
    }

    public double getDx() {
        return dx;
    }

    public boolean isIsFxSet() {
        return isFxSet;
    }

    public void setIsFxSet(boolean isFxSet) {
        this.isFxSet = isFxSet;
    }

    public boolean isIsFySet() {
        return isFySet;
    }

    public void setIsFySet(boolean isFySet) {
        this.isFySet = isFySet;
    }

    
    public void setDx(double dx) {
        this.dx = dx;
    }

    public double getDy() {
        return dy;
    }

    public void setDy(double dy) {
        this.dy = dy;
    }
    

    public boolean isxRestrained() {
        return xRestrained;
    }

    public void setxRestrained(boolean xRestrained) {
        this.xRestrained = xRestrained;
    }

    public boolean isyRestrained() {
        return yRestrained;
    }

    public void setyRestrained(boolean yRestrained) {
        this.yRestrained = yRestrained;
    }
    
    public int getxNo() {
        return xNo;
    }

    public void setxNo(int xNo) {
        this.xNo = xNo;
    }

    public int getyNo() {
        return yNo;
    }

    public void setyNo(int yNo) {
        this.yNo = yNo;
    }
    
    Joint(double x,double y){
        this.x=x;
        this.y=y;
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }
    
    public String toString(){
        return (this.x+","+this.y+" xNo="+this.getxNo()+" yNo="+this.getyNo());
    }
    
}
