package RSGridWorld;

import java.util.List;
import java.util.ArrayList;


public class TreeNode<T> {
    private List<TreeNode<T>> children = new ArrayList<TreeNode<T>>();
    private TreeNode<T> parent = null;
    private T data = null;

    public TreeNode(T data) {
        this.data = data;
    }

    public TreeNode(T data, TreeNode<T> parent) {
        this.data = data;
        this.parent = parent;
    }

    public List<TreeNode<T>> getChildren() {
        return children;
    }

    public void setParent(TreeNode<T> parent) {
        this.parent = parent;
    }

    public void addChild(T data) {
        TreeNode<T> child = new TreeNode<T>(data);
        child.setParent(this);
        this.children.add(child);
    }

    public void addChild(TreeNode<T> child) {
        child.setParent(this);
        this.children.add(child);
    }

    public T getData() {
        return this.data;
    }

    public void setData(T data) {
        this.data = data;
    }

    public boolean isRoot() {
        return (this.parent == null);
    }

    public boolean isLeaf() {
        if(this.children.size() == 0) 
            return true;
        else 
            return false;
    }

    public void removeParent() {
        this.parent = null;
    }

    private void printChildren(TreeNode<T> node){
        List<TreeNode<T>> children = node.getChildren();
        for(TreeNode<T> nodes : children)
            printChildren(nodes);
        System.out.println("Node data: " + node.getData());
    }

    public void printTree(){
        printChildren(this);
    }
}