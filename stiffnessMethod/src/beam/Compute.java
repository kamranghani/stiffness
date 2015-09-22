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
    // matrix of known loads
    double[][] load;
    //matrix of know displacements
    double [][] disp;

    public Compute(Model mo) {
        this.model = mo;
        model.populateJointAl();
        //model.getJointAl().get(0).setzNo(3);
        //model.getJointAl().get(0).setyNo(6);
        //model.getJointAl().get(2).setzNo(1);
        //model.getJointAl().get(2).setyNo(4);
        size = model.getJointAl().size();
        System.out.println("size is=" + size);
        this.sMat = new double[size * 2][size * 2];  //multiply by 2 because of two degree of freedom     
        this.populateSMat(model);
        //construct K11 and K22 matrix for load and displacement calculation
        this.extractK11_K21_K12_K22(model);
        
        this.calcUnknowns();
        //calculate member forces
        //this.calcMemberForces();
        this.printJoints(model);
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
            sMat[fy - 1][nz - 1]+= msmat[2][1];
            sMat[fy - 1][fy - 1]+= msmat[2][2];
            sMat[fy - 1][fz - 1] += msmat[2][3];
            sMat[fz - 1][ny - 1]+= msmat[3][0];
            sMat[fz - 1][nz - 1] += msmat[3][1];
            sMat[fz - 1][fy - 1] += msmat[3][2];
            sMat[fz - 1][fz - 1] += msmat[3][3];


            
             printMAT(msmat);
            
             System.out.println("--------------------------------------");
             System.out.println("---------------Another member---------");
             System.out.println("--------------------------------------");
             
        }
        printMAT(sMat);

        //  System.out.println("--------------------------------------");
        // System.out.println("---------------Another member---------");
        // System.out.println("--------------------------------------");
    }

    /**
     * for constructing member stiffness matrix for beam. Note that it is almost
     * the same as for truss but here we don't need lambda expressions as the x
     * and x' axes are in same directions and vice versa We compute displacement
     * due to shear in y direction and moments in z direction in case of beam.
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
        mSMat[0][0] = 12 / Math.pow(length, 3)*m.getE()*m.getI();
        mSMat[0][1] = 6 / Math.pow(length, 2)*m.getE()*m.getI();
        mSMat[0][2] = -12 / Math.pow(length, 3)*m.getE()*m.getI();
        mSMat[0][3] = 6 / Math.pow(length, 2)*m.getE()*m.getI();
        mSMat[1][0] = 6 / Math.pow(length, 2)*m.getE()*m.getI();
        mSMat[1][1] = 4 / length*m.getE()*m.getI();
        mSMat[1][2] = -6 / Math.pow(length, 2)*m.getE()*m.getI();
        mSMat[1][3] = 2 / length*m.getE()*m.getI();
        mSMat[2][0] = -12 / Math.pow(length, 3)*m.getE()*m.getI();
        mSMat[2][1] = -6 / Math.pow(length, 2)*m.getE()*m.getI();
        mSMat[2][2] = 12 / Math.pow(length, 3)*m.getE()*m.getI();
        mSMat[2][3] = -6 / Math.pow(length, 2)*m.getE()*m.getI();
        mSMat[3][0] = 6 / Math.pow(length, 2)*m.getE()*m.getI();
        mSMat[3][1] = 2 / length*m.getE()*m.getI();
        mSMat[3][2] = -6 / Math.pow(length, 2)*m.getE()*m.getI();
        mSMat[3][3] = 4 / length*m.getE()*m.getI();

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
                System.out.printf("%1.5f", m[i][j]);
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
    @SuppressWarnings("ManualArrayToCollectionCopy")
    private void extractK11_K21_K12_K22(Model mo) {
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
        int count = mo.getJointAl().size() * 2 - countR;
        //Construct a load matrix
        load = new double[count][1];

        // We have Qk=K11 Du+K12 Dk and 
        //         Qu=K21 Du+K22 Dk    where subscript u =unknow and k=known
        //based on no of known loads create the K11 and K12 matrices
        //If isIsFzSet then get the joint zNo for i and add values and vice versa
        //
        this.K11 = new double[count][count];
        
        this.K12 = new double[count][(mo.getJointAl().size() * 2) - count];
        this.K21 = new double[(mo.getJointAl().size() * 2) - count][count];
        
        this.K22=new double[(mo.getJointAl().size() * 2) - count][(mo.getJointAl().size() * 2) - count];
        int k = 0; //temporary counter
        int m = 0; //temporary counter;
        
        for (int i=0;i<count;i++){
            for (int j=0;j<count;j++){
                K11[i][j]=this.sMat[i][j];
            }
        }
        
        for (int i=0;i<count;i++){
            for (int j=count;j<(count+countR);j++){
                K12[i][m]=this.sMat[i][j];
                m++;
            } 
            m=0;
            
    }
        for (int i=count;i<count+countR;i++){
            for (int j=0;j<count;j++){
                K21[m][j]=this.sMat[i][j];
            }
                           m++;
        }
         m=0;

        for (int i=count;i<count+countR;i++){
            for (int j=count;j<count+countR;j++){
                K22[k][m]=this.sMat[i][j];
                
                m++;
            }
            k++;
            m=0;
        }
        
        
        //for storing known loads
        for (int i = 0; i < mo.getJointAl().size(); i++) {
            int x = mo.getJointAl().get(i).getzNo() - 1;
            int y = mo.getJointAl().get(i).getyNo() - 1;
            if (mo.getJointAl().get(i).isIsFzSet()) {
                load[x][0] = mo.getJointAl().get(i).getFz();
            }
            
            if (mo.getJointAl().get(i).isIsFySet()) {

                load[y][0] = mo.getJointAl().get(i).getFy();
            }
        }
        
        /*
               
        for (int i = 0; i < mo.getJointAl().size(); i++) {
            int x = mo.getJointAl().get(i).getzNo() - 1;
            int y = mo.getJointAl().get(i).getyNo() - 1;
            
            if (mo.getJointAl().get(i).isIsFzSet()) {
                load[x][0] = mo.getJointAl().get(i).getFz();
                for (int j = 0; j < count; j++) {
                    K11[x][j] = this.sMat[x][j];
                }
                for (int j = count; j < (count + countR); j++) {
                    K12[x][m] = this.sMat[x][j];
                    m++;
                }
                m = 0;  //reset counter for K12 
            } else {
                for (int j = 0; j < countR; j++) {
                    K21[k][j] = this.sMat[x][j];

                }
                for (int j = count; j < (count + countR); j++) {
                    K22[k][m] = this.sMat[x][j];
                    m++;
                }
                m = 0;

                k++;
            }

            if (mo.getJointAl().get(i).isIsFySet()) {

                load[y][0] = mo.getJointAl().get(i).getFy();
                for (int j = 0; j < count; j++) {
                    K11[y][j] = this.sMat[y][j];
                }
                for (int j = count; j < (count + countR); j++) {
                    K12[y][m] = this.sMat[y][j];
                    m++;
                }
                m = 0;
            } else {
                for (int j = 0; j < count; j++) {
                    K21[k][j] = this.sMat[y][j];
                }
                for (int j = count; j < (count + countR); j++) {
                    K22[k][m] = this.sMat[y][j];
                    m++;
                }
                m = 0;
                k++;
            }

        }
*/    }
    
    public void calcUnknowns(){
        
        
        
        System.out.println("-------K11 Matrix-----");
        printMAT(K11);
        System.out.println("-------K12 Matrix-----");
        printMAT(K12);
        System.out.println("-------K21 Matrix-----");
        printMAT(K21);
        System.out.println("-------K22 Matrix-----");
        printMAT(K22);
        
        //following loop is for populating the known displacement matrix .i.e.Dk
        int k=0;
        disp=new double [K12[0].length][1];
          for(int i=0;i<this.model.getJointAl().size();i++){
              if(this.model.getJointAl().get(i).isyDisplaced()){
                  this.disp[k][0]=this.model.getJointAl().get(i).getDy();
                  k++;
              }
              
              if(this.model.getJointAl().get(i).iszDisplaced()){
                  this.disp[k][0]=this.model.getJointAl().get(i).getDz();
                  k++;
              }
          }
          
         // printMAT(disp);
         Matrix k11 = new Matrix(K11);
        // System.out.println("K12 MATRIX");
         //printMAT(K12);
        Matrix k12 = new Matrix(K12);
        Matrix k21=new Matrix(K21);
        Matrix k22=new Matrix(K22);
         
         Matrix qk = new Matrix(load);
         Matrix dk=new Matrix(disp);
         System.out.println("displacements");
         printMAT(dk.getArray());
         
         Matrix du =((k11.inverse()).times(qk)).minus(((k11.inverse()).times(k12).times(dk))) ;  // Du=K11^1 *qk-k11^1*k12*dk (where k11^1 is inverse inverse of k11)
        
         System.out.println("-------Displacements are-----");
         printMAT(du.getArray());
         printMAT(K21);
        
         System.out.println("-------reactions are-----");
        
        Matrix q1=k21.times(du);
         //Matrix q1=(k21.times(du)).plus(k22.times(dk));
         printMAT(q1.getArray());  
         printMAT(load);
    }

    /**
     * Method is used for calculating member forces after finding joint
     * displacements and support reactions
     */
    
    
    public void calcMemberForces() {

    }
    
    public void printJoints(Model model){
    int j=1;
        for (int k=0;k<model.getJointAl().size();k++){
            System.out.println("j"+j++ +"="+model.getJointAl().get(k).toString());
        }

    }
}
