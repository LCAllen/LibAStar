using System;
using System.Collections.Generic;
using System.Linq;

namespace LibAStar
{
	/// <summary>
	/// Helper class for A* Pathfinding..
	/// </summary>
	public class AStar
	{
		/// <summary>
		/// Find a path through a 2d array.
		/// </summary>
		/// <returns>
		/// A Tuple containing:
		/// Item1: A list of nodes (path).
		/// Item2: A cost associated with that path.
		/// </returns>
		/// <param name="map">A 2 dimensional array of ints. The higher the number, the more difficult the terrain. 0 is impassable.</param>
		/// <param name="x">Starting location on x axis.</param>
		/// <param name="y">Starting location on y axis.</param>
		/// <param name="endX">Ending location on x axis..</param>
		/// <param name="endY">Ending location on y axis..</param>
		public static Tuple<List<Node>, int> findPath(int[,] map, int x, int y, int endX, int endY)
		{
			List<Node> returnList = new List<Node>();
			List<Node> openList = new List<Node>();
			List<Node> closedList = new List<Node>();

			if (map[x, y] == 0 || map[endX, endY] == 0)
			{
				return Tuple.Create((List<Node>)null, 0);
			}

			int numTiles = map.GetLength(0) * map.GetLength(1);
			int cost = 0;
			bool found = false;
			Node start = new Node(x, y, map[y, x], 0);
			start.calculateScores(endX, endY);
			Node current = null;

			// start by adding the original position to the open list
			openList.Add(start);

			while (openList.Count > 0)
			{
				// Get the square with the lowest F score
				var lowest = openList.Min(l => l.getF());
				current = openList.First(l => l.getF() == lowest);

				// Mark as searched
				closedList.Add(current);
				openList.Remove(current);

				// Current node is target node
				if (closedList.FirstOrDefault(l => l.getX() == endX && l.getY() == endY) != null)
				{
					cost = current.getG();
					found = true;
					break;
				}

				// Get all neighbors of current node, and calculate their scores
				List<Node> neighbors = getNeighbors(map, current);
				foreach (Node neighbor in neighbors)
				{
					neighbor.calculateScores(endX, endY);
				}

				// 
				foreach (Node neighbor in neighbors)
				{
					// Check if the neighbor has already been added to a list
					if (closedList.FirstOrDefault(l => l.getX() == neighbor.getX()
												  && l.getY() == neighbor.getY()) != null)
					{
						continue;
					}
					if (openList.FirstOrDefault(l => l.getX() == neighbor.getX()
												&& l.getY() == neighbor.getY()) == null)
					{
						
						neighbor.setParent(current);
						openList.Insert(0, neighbor);
					}

					// If it has not been added to a list, check its score
					else
					{
						// Update the "path" if the score is better than the current node
						current.calculateScores(endX, endY);
						if (current.getG() + neighbor.getH() < neighbor.getF())
						{
							neighbor.setParent(current);
						}
					}
				}
			}

			// Backtrack the path, and add each node to the return list if found
			// Return null if no path found
			while (current != null)
			{
				if (found == true)
				{
					returnList.Add(current);
					current = current.getParent();
				}
				else
				{
					return null;
				}
			}

			return Tuple.Create(returnList, cost);
		}

		// Find all passable neighbors of the current node, and return them in a list
		private static List<Node> getNeighbors(int[,] map, Node node)
		{
			List<Node> neighbors = new List<Node>();

			if (isValidPoint(map, node.getX() - 1, node.getY()))
			{
				neighbors.Add(new Node(node.getX() - 1, node.getY(), map[node.getY(), node.getX() - 1], node.getG()));
			}

			if (isValidPoint(map, node.getX() + 1, node.getY()))
			{
				neighbors.Add(new Node(node.getX() + 1, node.getY(), map[node.getY(), node.getX() + 1], node.getG()));
			}

			if (isValidPoint(map, node.getX(), node.getY() - 1))
			{
				neighbors.Add(new Node(node.getX(), node.getY() - 1, map[node.getY() - 1, node.getX()], node.getG()));
			}

			if (isValidPoint(map, node.getX(), node.getY() + 1))
			{
				neighbors.Add(new Node(node.getX(), node.getY() + 1, map[node.getY() + 1, node.getX()], node.getG()));
			}

			return neighbors;
		}

		// Check if the given node is passable
		private static bool isValidPoint(int[,] map, int x, int y)
		{
			return !(x < 0 || x >= map.GetLength(0) || y < 0 || y >= map.GetLength(1)) && (map[y, x] != 0);
		}
	}

	/// <summary>
	/// Datatype for holding tile information in A* Pathfinding.
	/// </summary>
	public class Node
	{
		private int x = -1;
		private int y = -1;
		private int difficulty = -1;
		private int g = -1;
		private int h = -1;
		private int f = -1;
		private Node parent = null;

		/// <summary>
		/// Constructor for Node datatype.
		/// </summary>
		public Node(int x, int y, int difficulty, int g)
		{
			this.x = x;
			this.y = y;
			this.difficulty = difficulty;
			this.g = g + difficulty;
		}

		private void calculateF()
		{
			f = g + h;
		}

		/// <summary>
		/// Calculates the H and F scores of the selected node..
		/// </summary>
		public void calculateScores(int targetX, int targetY)
		{
			calculateH(targetX, targetY);
			calculateF();
		}

		private void calculateH(int targetX, int targetY)
		{
			h = Math.Abs(targetX - x) + Math.Abs(targetY - y);
		}

		// Getters and Setters
		/// <summary>
		/// Returns the X coordinate of the node.
		/// </summary>
		public int getX()
		{
			return x;
		}

		/// <summary>
		/// Returns the Y coordinate of the node.
		/// </summary>
		public int getY()
		{
			return y;
		}

		/// <summary>
		/// Returns the difficulty of the node.
		/// </summary>
		public int getDifficulty()
		{
			return difficulty;
		}

		/// <summary>
		/// Returns the G value of the node.
		/// </summary>
		public int getG()
		{
			return g;
		}

		/// <summary>
		/// Returns the H value of the node.
		/// </summary>
		public int getH()
		{
			return h;
		}

		/// <summary>
		/// Returns the F value of the node.
		/// </summary>
		public int getF()
		{
			return f;
		}

		/// <summary>
		/// Returns the parent of the node.
		/// </summary>
		public Node getParent()
		{
			return parent;
		}

		/// <summary>
		/// Sets the parent of the node.
		/// </summary>
		public void setParent(Node parent)
		{
			this.parent = parent;
		}
	}
}
