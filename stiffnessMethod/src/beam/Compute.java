package beam;

/**
 *
 * @author Kamran
 */
import Jama.Matrix;

public class Compute {

    private Model model;
    private int size = 0;
    //global stiffness matrix
    private double[][] sMat;
    //for finding displacement and unknow reaction (same term as in Hibbeler 8th addition page 552
    private double[][] K11;
    private double[][] K21;
    private double[][] K12;
    private double[][] K22;

    public Compute(Model mo) {
        this.model = mo;
        model.populateJointAl();
        size = model.getJointAl().size();
        System.out.println("size is=" + size);
        this.sMat = new double[size * 2][size * 2];  //multiply by 2 because of two degree of freedom     
        this.populateSMat(mo);
        //construct K11 and K22 matrix for load and displacement calculation
        this.extractK11_K21(mo);
        //calculate member forces
        //this.calcMemberForces();

    }

    public void setSize(int s) {
        this.size = s;
    }

    public int getSize() {
        return size;
    }

    /**
     * the following method is written to construct the golbal stiffness matrix
     */
    private void populateSMat(Model mo) {
        //member stiffness matrix
        //double[][] mSMat=new double[4][4];
        for (int i = 0; i < mo.getMemberAl().size(); i++) {
            //construct member stiffness member      
            double[][] msmat;
            msmat = constructMSMat(mo.getMemberAl().get(i));
            //since we know the number of elements to be entered
            //and we also know the joint numbers of each member
            //therefore we will enter the values in global stiffness
            // matrix using the joint numbers of the member

            int nz = mo.getMemberAl().get(i).getN().getzNo(); //near end x
            int ny = mo.getMemberAl().get(i).getN().getyNo();
            int fz = mo.getMemberAl().get(i).getF().getzNo();
            int fy = mo.getMemberAl().get(i).getF().getyNo();

            sMat[ny - 1][ny - 1] += msmat[0][0];
            sMat[ny - 1][nz - 1] += msmat[0][1];
            sMat[ny - 1][fy - 1] += msmat[0][2];
            sMat[ny - 1][fz - 1] += msmat[0][3];
            sMat[nz - 1][ny - 1] += msmat[1][0];
            sMat[nz - 1][nz - 1] += msmat[1][1];
            sMat[nz - 1][fy - 1] += msmat[1][2];
            sMat[nz - 1][fz - 1] += msmat[1][3];
            sMat[fy - 1][ny - 1] += msmat[2][0];
            sMat[fy - 1][nz - 1] += msmat[2][1];
            sMat[fy - 1][fy - 1] += msmat[2][2];
            sMat[fy - 1][fz - 1] += msmat[2][3];
            sMat[fz - 1][ny - 1] += msmat[3][0];
            sMat[fz - 1][nz - 1] += msmat[3][1];
            sMat[fz - 1][fy - 1] += msmat[3][2];
            sMat[fz - 1][fz - 1] += msmat[3][3];


         /*
             printMAT(msmat);
            
             System.out.println("--------------------------------------");
             System.out.println("---------------Another member---------");
             System.out.println("--------------------------------------");
            */ 
        }
       printMAT(sMat);

        //  System.out.println("--------------------------------------");
        // System.out.println("---------------Another member---------");
        // System.out.println("--------------------------------------");

    }

    /**
     * for constructing member stiffness matrix for beam. Note that it is 
     * almost the same as for truss but here we don't need lambda expressions
     * as the x and x' axes are in same directions and vice versa
     * We compute displacement due to shear in y direction and moments in z direction 
     * in case of beam.
     *
     */
    public double[][] constructMSMat(Member m) {
        //caculate length
        
        double length = Math.sqrt((m.getF().getX() - m.getN().getX()) * (m.getF().getX() - m.getN().getX())
                + (m.getF().getY() - m.getN().getY()) * (m.getF().getY() - m.getN().getY()));
        //double lambda[] = cosine(m.getN().getX(), m.getF().getX(), m.getN().getY(), m.getF().getY());
        double[][] mSMat;
        mSMat = new double[4][4];
        //To reduce complexity, we avoided the loops. The following can
        // alse be entered using loops
        mSMat[0][0] = 12/Math.pow(length, 3);
        mSMat[0][1] = 6/Math.pow(length, 2);
        mSMat[0][2] = -12/Math.pow(length, 3);
        mSMat[0][3] = 6/Math.pow(length, 2);
        mSMat[1][0] = 6/Math.pow(length, 2);
        mSMat[1][1] = 4/length;
        mSMat[1][2] = -6/Math.pow(length, 2);
        mSMat[1][3] = 2/length;
        mSMat[2][0] = -12/Math.pow(length, 3);
        mSMat[2][1] = -6/Math.pow(length, 2);
        mSMat[2][2] = 12/Math.pow(length, 3);
        mSMat[2][3] = -6/Math.pow(length, 2);
        mSMat[3][0] = 6/Math.pow(length, 2);
        mSMat[3][1] = 2/length;
        mSMat[3][2] = -6/Math.pow(length, 2);
        mSMat[3][3] = 4/length;

        //divide each element by Length L
        
        return mSMat;
    }

    /**
     * Method is used to calculate cosine called lambda in Hibbeler book.
     *
     * @param xn near joint x component
     * @param xf far joint x component
     * @param yn near joint y component
     * @param yf far joint y component
     * @return
     */
    public double[] cosine(double xn, double xf, double yn, double yf) {

        //length of member
        double length = Math.sqrt((xf - xn) * (xf - xn) + (yf - yn) * (yf - yn));
        double cosineX = (xf - xn) / length;
        double cosineY = (yf - yn) / length;

        double cosineAr[] = {cosineX, cosineY, length};//or call is lambdaX and lambdaY in accordance with hibbeler book.
        return cosineAr;
    }

    //for printing two dimensional array.
    public void printMAT(double[][] m) {
        for (int i = 0; i < m.length; i++) {
            for (int j = 0; j < m[i].length; j++) {
                System.out.printf("%1.3f", m[i][j]);
                System.out.print("  ");
            }
            System.out.println();
        }
    }

    //For calculating unknown displacements.
    /**
     * Compute the displacements cause by applied loads
     *
     * @param mo
     */
    private void extractK11_K21(Model mo) {
        // count the no of unknow displacements i.e, the values for which isyRestrained() and iszRestrained()
        //has been set.

        int countR = 0;
        for (int i = 0; i < mo.getJointAl().size(); i++) {
            if (mo.getJointAl().get(i).isyRestrained()) {
                countR = countR + 1;
            }
            if (mo.getJointAl().get(i).iszRestrained()) {
                countR = countR + 1;
            }
        }
        
        //From countR find count i.e., the no of load applied. Note based on support condition
        //we will take loads equal to zero if not a load is not applied
        int count=mo.getJointAl().size()*2-countR;
        //Construct a load matrix
        double[][] load = new double[count][1];


        //based on no of known loads create the K11 and K21 matrices
        //If isIsFzSet then get the joint zNo for i and add values and vice versa
        //till 
        this.K11 = new double[count][count];
        this.K21 = new double[(mo.getJointAl().size() * 2) - count][ count];
        int k=0; //counter variable for K21
        for (int i = 0; i < mo.getJointAl().size(); i++) {
            int x = mo.getJointAl().get(i).getzNo() - 1;
            int y = mo.getJointAl().get(i).getyNo() - 1;
            if (mo.getJointAl().get(i).isIsFzSet()) {

                load[x][0] = mo.getJointAl().get(i).getFz(); 
                for (int j = 0; j < count; j++) {
                    K11[x][j] = this.sMat[x][j];
                }
            } else {
                for (int j = 0; j < countR; j++) {
                    K21[k][j] = this.sMat[x][j];
                    
                }
                k++;
            }

                if (mo.getJointAl().get(i).isIsFySet()) {

                    load[y][0] = mo.getJointAl().get(i).getFy();
                    for (int j = 0; j < count; j++) {
                        K11[y][j] = this.sMat[y][j];
                    }
                } else {
                    for (int j = 0; j < count; j++) {
                        K21[k][j] = this.sMat[y][j];
                        
                    }
                    k++;
                }
            
        }
        //System.out.println("-------K11 Matrix-----");
        printMAT(K11);
        Matrix k11 = new Matrix(K11);
        Matrix q = new Matrix(load);
        Matrix d = k11.inverse().times(q);
        
        System.out.println("-------Displacements are-----");
        printMAT(d.getArray());
        printMAT(K21);
        
        System.out.println("-------reactions are-----");
        
        Matrix k21=new Matrix(K21);
        Matrix q1=k21.times(d);
        printMAT(q1.getArray());
    }  
    
    
   /**
    * Method is used for calculating member forces after finding
    * joint displacements and support reactions
    */
    public void calcMemberForces(){
        
    }
}
